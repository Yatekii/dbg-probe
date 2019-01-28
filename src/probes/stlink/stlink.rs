// from . import STLinkException
// from .constants import (Commands, Status, SWD_FREQ_MAP, JTAG_FREQ_MAP)
// from ...core import exceptions
// from ...coresight import dap
// import logging
// import struct
// import six
// from enum import Enum

use crate::protocol::WireProtocol;
use crate::probes::stlink::constants::JTagFrequencyToDivider;
use crate::probes::stlink::constants::SwdFrequencyToDelayCount;
use crate::probes::stlink::usb_interface::{
    TIMEOUT,
    STLinkUSBDevice
};
use super::constants::{
    commands,
    Status
};
use crate::common::BytesTo;

type AccessPort = u8;

pub struct STLink<'a> {
    device: STLinkUSBDevice<'a>,
    hw_version: u8,
    jtag_version: u32,
    protocol: WireProtocol,
}

pub enum STLinkError {
    USB(libusb::Error),
    JTAGNotSupportedOnProbe,
    ProbeFirmwareOutdated,
    VoltageDivisionByZero,
    UnknownMode,
    JTagDoesNotSupportMultipleAP,
    UnknownError,
    TransferFault(u32, u16),
    DataAlignmentError,
    Access16BitNotSupported,
    BlanksNotAllowedOnDPRegister,
    RegisterAddressMustBe16Bit,
}

impl<'a> STLink<'a> {
    
    /// Maximum number of bytes to send or receive for 32- and 16- bit transfers.
    /// 
    /// 8-bit transfers have a maximum size of the maximum USB packet size (64 bytes for full speed).
    const MAXIMUM_TRANSFER_SIZE: u32 = 1024;
    
    /// Minimum required STLink firmware version.
    const MIN_JTAG_VERSION: u32 = 24;
    
    /// Firmware version that adds 16-bit transfers.
    const MIN_JTAG_VERSION_16BIT_XFER: u32 = 26;
    
    /// Firmware version that adds multiple AP support.
    const MIN_JTAG_VERSION_MULTI_AP: u32 = 28;
    
    /// Port number to use to indicate DP registers.
    const DP_PORT: u16 = 0xffff;

    pub fn new(device: STLinkUSBDevice<'a>) -> Self {
        Self {
            device,
            hw_version: 0,
            jtag_version: 0,
            protocol: WireProtocol::Swd,
        }
    }
    
    pub fn open(&mut self) {
        self.device.open();
        self.enter_idle();
        self.get_version();
        self.get_target_voltage();
    }

    fn close(&mut self) {
        self.enter_idle();
        self.device.close();
    }

    fn get_version(&mut self) -> Result<(), STLinkError> {
        const HW_VERSION_SHIFT: u8 = 12;
        const HW_VERSION_MASK: u8 = 0x0F;
        const JTAG_VERSION_SHIFT: u8 = 6;
        const JTAG_VERSION_MASK: u8 = 0x3F;
        // GET_VERSION response structure:
        //   Byte 0-1:
        //     [15:12] Major/HW version
        //     [11:6]  JTAG/SWD version
        //     [5:0]   SWIM or MSC version
        //   Byte 2-3: ST_VID
        //   Byte 4-5: STLINK_PID
        let mut buf = [0; 12];
        match self.device.write(vec![commands::GET_VERSION], &[], &mut buf, TIMEOUT) {
            Ok(_) => {
                let version: u16 = ((buf[1] as u16) << 8) | buf[0] as u16;
                self.hw_version = (version >> HW_VERSION_SHIFT) as u8 & HW_VERSION_MASK;
                self.jtag_version = (version >> JTAG_VERSION_SHIFT) as u32 & JTAG_VERSION_MASK as u32;
            },
            Err(e) => return Err(STLinkError::USB(e))
        }
        
        // For STLinkV3 we must use the extended get version command.
        if self.hw_version >= 3 {
            // GET_VERSION_EXT response structure (byte offsets) {
            //  0: HW version
            //  1: SWIM version
            //  2: JTAG/SWD version
            //  3: MSC/VCP version
            //  4: Bridge version
            //  5-7: reserved
            //  8-9: ST_VID
            //  10-11: STLINK_PID
            match self.device.write(vec![commands::GET_VERSION_EXT], &[], &mut buf, TIMEOUT) {
                Ok(_) => {
                    let version: u32 = (&buf[0..4]).to_u32();
                    self.jtag_version = version;
                },
                Err(e) => return Err(STLinkError::USB(e))
            }
        }
            
        // Check versions.
        if self.jtag_version == 0 {
            return Err(STLinkError::JTAGNotSupportedOnProbe);
        }
        if self.jtag_version < Self::MIN_JTAG_VERSION {
            return Err(STLinkError::ProbeFirmwareOutdated)
        }

        Ok(())
    }

    fn get_target_voltage(&mut self) -> Result<u32, STLinkError> {
        let mut buf = [0; 8];
        match self.device.write(vec![commands::GET_TARGET_VOLTAGE], &[], &mut buf, TIMEOUT) {
            Ok(_) => {
                let a0 = (&buf[0..4]).to_u32() as f32;
                let a1 = (&buf[5..8]).to_u32() as f32;
                if a0 != 0.0 {
                    Ok((2.0 * a1 * 1.2 / a0) as u32)
                } else {
                    Err(STLinkError::VoltageDivisionByZero)
                }
            },
            Err(e) => Err(STLinkError::USB(e))
        }
    }

    fn enter_idle(&mut self) -> Result<(), STLinkError> {
        let mut buf = [0; 2];
        match self.device.write(vec![commands::GET_CURRENT_MODE], &[], &mut buf, TIMEOUT) {
            Ok(_) => {
                if buf[0] == commands::DEV_DFU_MODE {
                    self.device.write(vec![commands::DFU_COMMAND, commands::DFU_EXIT], &[], &mut[], TIMEOUT)
                               .map_err(|e| STLinkError::USB(e))
                } else if buf[0] == commands::DEV_JTAG_MODE {
                    self.device.write(vec![commands::JTAG_COMMAND, commands::JTAG_EXIT], &[], &mut[], TIMEOUT)
                               .map_err(|e| STLinkError::USB(e))
                } else if buf[0] == commands::DEV_SWIM_MODE {
                    self.device.write(vec![commands::SWIM_COMMAND, commands::SWIM_EXIT], &[], &mut[], TIMEOUT)
                               .map_err(|e| STLinkError::USB(e))
                } else {
                    Err(STLinkError::UnknownMode)
                }
            },
            Err(e) => Err(STLinkError::USB(e))
        }
    }

    fn set_swd_frequency(&mut self, frequency: SwdFrequencyToDelayCount) -> Result<(), STLinkError> {
        let mut buf = [0; 2];
        self.device.write(vec![commands::JTAG_COMMAND, commands::SWD_SET_FREQ, frequency as u8], &[], &mut buf, TIMEOUT)
                   .map_err(|e| STLinkError::USB(e))?;
        Self::check_status(&buf)
    }

    fn set_jtag_frequency(&mut self, frequency: JTagFrequencyToDivider) -> Result<(), STLinkError> {
        let mut buf = [0; 2];
        self.device.write(vec![commands::JTAG_COMMAND, commands::JTAG_SET_FREQ, frequency as u8], &[], &mut buf, TIMEOUT)
                   .map_err(|e| STLinkError::USB(e))?;
        Self::check_status(&buf)
    }

    fn enter_debug(&mut self, protocol: WireProtocol) -> Result<(), STLinkError> {
        self.enter_idle();
        
        let param = match protocol {
            WireProtocol::Jtag => commands::JTAG_ENTER_SWD,
            WireProtocol::Swd => commands::JTAG_ENTER_JTAG_NO_CORE_RESET
        };

        let mut buf = [0; 2];
        self.device.write(vec![commands::JTAG_COMMAND, commands::JTAG_ENTER2, param, 0], &[], &mut buf, TIMEOUT)
                   .map_err(|e| STLinkError::USB(e))?;
        self.protocol = protocol;
        return Self::check_status(&buf);
    }
    
    fn open_ap(&mut self, apsel: AccessPort) -> Result<(), STLinkError> {
        if self.jtag_version < Self::MIN_JTAG_VERSION_MULTI_AP as u32 {
            return Err(STLinkError::JTagDoesNotSupportMultipleAP);
        }
        let mut buf = [0; 2];
        self.device.write(vec![commands::JTAG_COMMAND, commands::JTAG_INIT_AP, apsel, commands::JTAG_AP_NO_CORE], &[], &mut buf, TIMEOUT)
                   .map_err(|e| STLinkError::USB(e))?;
        return Self::check_status(&buf)
    }
    
    fn close_ap(&mut self, apsel: AccessPort) -> Result<(), STLinkError> {
        if self.jtag_version < Self::MIN_JTAG_VERSION_MULTI_AP as u32 {
            return Err(STLinkError::JTagDoesNotSupportMultipleAP);
        }
        let mut buf = [0; 2];
        self.device.write(vec![commands::JTAG_COMMAND, commands::JTAG_CLOSE_AP_DBG, apsel], &[], &mut buf, TIMEOUT)
                   .map_err(|e| STLinkError::USB(e))?;
        return Self::check_status(&buf)
    }

    fn target_reset(&mut self) -> Result<(), STLinkError> {
        let mut buf = [0; 2];
        self.device.write(vec![commands::JTAG_COMMAND, commands::JTAG_DRIVE_NRST, commands::JTAG_DRIVE_NRST_PULSE], &[], &mut buf, TIMEOUT)
                   .map_err(|e| STLinkError::USB(e))?;
        return Self::check_status(&buf)
    }
    
    fn drive_nreset(&mut self, is_asserted: bool) -> Result<(), STLinkError> {
        let state = if is_asserted { commands::JTAG_DRIVE_NRST_LOW } else { commands::JTAG_DRIVE_NRST_HIGH };
        let mut buf = [0; 2];
        self.device.write(vec![commands::JTAG_COMMAND, commands::JTAG_DRIVE_NRST, state], &[], &mut buf, TIMEOUT)
                   .map_err(|e| STLinkError::USB(e))?;
        return Self::check_status(&buf)
    }
    
    fn check_status(status: &[u8]) -> Result<(), STLinkError> {
        if status[0] != Status::JtagOk as u8 {
            Err(STLinkError::UnknownError)
        } else {
            Ok(())
        }
    }

    fn clear_sticky_error(&mut self) -> Result<(), STLinkError> {
        // TODO: Implement this as soon as CoreSight is implemented
        // match self.protocol {
        //     WireProtocol::Jtag => self.write_dap_register(Self::DP_PORT, dap.DP_CTRL_STAT, dap.CTRLSTAT_STICKYERR),
        //     WireProtocol::Swd => self.write_dap_register(Self::DP_PORT, dap.DP_ABORT, dap.ABORT_STKERRCLR)
        // }
        Ok(())
    }
    
    fn read_mem(&mut self, mut addr: u32, mut size: u32, memcmd: u8, max: u32, apsel: AccessPort) -> Result<Vec<u8>, STLinkError> {
        let mut result = vec![];
        while size > 0 {
            let transfer_size = u32::min(size, max);
            
            let cmd = vec![
                commands::JTAG_COMMAND,
                memcmd,
                addr as u8 | 0xFF, (addr >> 8) as u8 | 0xFF, (addr >> 16) as u8 | 0xFF, (addr >> 24) as u8 | 0xFF,
                (transfer_size >> 0) as u8 | 0xFF, (transfer_size >> 8) as u8 | 0xFF,
                apsel
            ];
            let mut buf = Vec::with_capacity(transfer_size as usize);
            self.device.write(cmd, &[], buf.as_mut_slice(), TIMEOUT).map_err(|e| STLinkError::USB(e))?;
            result.extend(buf.into_iter());

            addr += transfer_size as u32;
            size -= transfer_size;
            
            // Check status of this read.
            let mut buf = [0; 12];
            self.device.write(vec![commands::JTAG_COMMAND, commands::JTAG_GETLASTRWSTATUS2], &[], &mut buf, TIMEOUT)
                       .map_err(|e| STLinkError::USB(e))?;
            let status = (&buf[0..2]).to_u16();
            let fault_address = (&buf[4..8]).to_u32();
            if if status == Status::JtagUnknownError as u16 { true }
            else if status == Status::SwdApFault as u16 { true }
            else if status == Status::SwdDpFault as u16 { true }
            else if status == Status::JtagOk as u16 { return Err(STLinkError::UnknownError) }
            else { false } {
                self.clear_sticky_error();
                return Err(STLinkError::TransferFault(fault_address, (transfer_size - (fault_address - addr)) as u16));
            }
        }
        Ok(result)
    }

    fn write_mem(&mut self, mut addr: u32, mut data: Vec<u8>, memcmd: u8, max: u32, apsel: AccessPort) -> Result<(), STLinkError> {
        while data.len() > 0 {
            let transfer_size = u32::min(data.len() as u32, max);
            let transfer_data = &data[0..transfer_size as usize];
            
            let cmd = vec![
                commands::JTAG_COMMAND,
                memcmd,
                addr as u8 | 0xFF, (addr >> 8) as u8 | 0xFF, (addr >> 16) as u8 | 0xFF, (addr >> 24) as u8 | 0xFF,
                (transfer_size >> 0) as u8 | 0xFF, (transfer_size >> 8) as u8 | 0xFF,
                apsel
            ];
            self.device.write(cmd, transfer_data, &mut [], TIMEOUT).map_err(|e| STLinkError::USB(e))?;

            addr += transfer_size as u32;
            data.drain(..transfer_size as usize);
            
            // Check status of this read.
            let mut buf = [0; 12];
            self.device.write(vec![commands::JTAG_COMMAND, commands::JTAG_GETLASTRWSTATUS2], &[], &mut buf, TIMEOUT)
                       .map_err(|e| STLinkError::USB(e))?;
            let status = (&buf[0..2]).to_u16();
            let fault_address = (&buf[4..8]).to_u32();
            if if status == Status::JtagUnknownError as u16 { true }
            else if status == Status::SwdApFault as u16 { true }
            else if status == Status::SwdDpFault as u16 { true }
            else if status == Status::JtagOk as u16 { return Err(STLinkError::UnknownError) }
            else { false } {
                self.clear_sticky_error();
                return Err(STLinkError::TransferFault(fault_address, (transfer_size - (fault_address - addr)) as u16));
            }
        }
        Ok(())
    }

    fn read_mem32(&mut self, addr: u32, size: u32, apsel: AccessPort) -> Result<Vec<u8>, STLinkError> {
        if (addr & 0x3) == 0 && (size & 0x3) == 0 {
            return self.read_mem(addr, size, commands::JTAG_READMEM_32BIT, Self::MAXIMUM_TRANSFER_SIZE, apsel);
        }
        Err(STLinkError::DataAlignmentError)
    }

    fn write_mem32(&mut self, addr: u32, data: Vec<u8>, apsel: AccessPort) -> Result<(), STLinkError> {
        if (addr & 0x3) == 0 && (data.len() & 0x3) == 0 {
            return self.write_mem(addr, data, commands::JTAG_WRITEMEM_32BIT, Self::MAXIMUM_TRANSFER_SIZE, apsel);
        }
        Err(STLinkError::DataAlignmentError)
    }

    fn read_mem16(&mut self, addr: u32, size: u32, apsel: AccessPort) -> Result<Vec<u8>, STLinkError> {
        if (addr & 0x1) == 0 && (size & 0x1) == 0 {
            if self.jtag_version < Self::MIN_JTAG_VERSION_16BIT_XFER {
                return Err(STLinkError::Access16BitNotSupported)
            }
            return self.read_mem(addr, size, commands::JTAG_READMEM_16BIT, Self::MAXIMUM_TRANSFER_SIZE, apsel);
        }
        Err(STLinkError::DataAlignmentError)
    }

    fn write_mem16(&mut self, addr: u32, data: Vec<u8>, apsel: AccessPort) -> Result<(), STLinkError> {
        if (addr & 0x1) == 0 && (data.len() & 0x1) == 0 {
            if self.jtag_version < Self::MIN_JTAG_VERSION_16BIT_XFER {
                return Err(STLinkError::Access16BitNotSupported)
            }
            return self.write_mem(addr, data, commands::JTAG_WRITEMEM_16BIT, Self::MAXIMUM_TRANSFER_SIZE, apsel);
        }
        Err(STLinkError::DataAlignmentError)
    }

    fn read_mem8(&mut self, addr: u32, size: u32, apsel: AccessPort) -> Result<Vec<u8>, STLinkError> {
        self.read_mem(addr, size, commands::JTAG_READMEM_8BIT, Self::MAXIMUM_TRANSFER_SIZE, apsel)
    }

    fn write_mem8(&mut self, addr: u32, data: Vec<u8>, apsel: AccessPort) -> Result<(), STLinkError> {
        self.write_mem(addr, data, commands::JTAG_WRITEMEM_8BIT, Self::MAXIMUM_TRANSFER_SIZE, apsel)
    }
    
    fn read_dap_register(&mut self, port: u16, addr: u32) -> Result<u32, STLinkError> {
        if (addr & 0xf0) == 0 || port != Self::DP_PORT {
            if (addr >> 16) == 0 {
                let cmd = vec![
                    commands::JTAG_COMMAND,
                    commands::JTAG_READ_DAP_REG,
                    (port & 0xFF) as u8,
                    ((port >> 8) & 0xFF) as u8,
                    (addr & 0xFF) as u8,
                    ((addr >> 8) & 0xFF) as u8
                ];
                let mut buf = [0; 8];
                self.device.write(cmd, &[], &mut buf, TIMEOUT)
                        .map_err(|e| STLinkError::USB(e))?;
                Self::check_status(&buf)?;
                Ok((&buf[0..4]).to_u32())
            } else {
                Err(STLinkError::RegisterAddressMustBe16Bit)
            }
        } else {
            Err(STLinkError::BlanksNotAllowedOnDPRegister)
        }
    }
    
    fn write_dap_register(&mut self, port: u16, addr: u32, value: u32) -> Result<(), STLinkError> {
        if (addr & 0xf0) == 0 || port != Self::DP_PORT {
            if (addr >> 16) == 0 {
                let cmd = vec![
                    commands::JTAG_COMMAND,
                    commands::JTAG_WRITE_DAP_REG,
                    (port & 0xFF) as u8,
                    ((port >> 8) & 0xFF) as u8,
                    (addr & 0xFF) as u8,
                    ((addr >> 8) & 0xFF) as u8,
                    (value & 0xFF) as u8,
                    ((value >> 8) & 0xFF) as u8,
                    ((value >> 16) & 0xFF) as u8,
                    ((value >> 24) & 0xFF) as u8,
                ];
                let mut buf = [0; 8];
                self.device.write(cmd, &[], &mut buf, TIMEOUT)
                        .map_err(|e| STLinkError::USB(e))?;
                Self::check_status(&buf)?;
                Ok(())
            } else {
                Err(STLinkError::RegisterAddressMustBe16Bit)
            }
        } else {
            Err(STLinkError::BlanksNotAllowedOnDPRegister)
        }
    }
}