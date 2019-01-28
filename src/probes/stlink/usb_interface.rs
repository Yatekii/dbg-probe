use std::time::Duration;
use libusb::{
    DeviceHandle,
    Context,
    Error,
    Device
};
use log::{info, trace, warn};
use lazy_static::lazy_static;

use std::collections::HashMap;

/// The USB Command packet size.
const CMD_LEN: usize = 16;

/// The USB VendorID.
const USB_VID: u16 = 0x0483;

pub const TIMEOUT: Duration = Duration::from_millis(1000);

lazy_static! {
    /// Map of USB PID to firmware version name and device endpoints.
    static ref USB_PID_EP_MAP: HashMap<u16, STLinkInfo> = {
        let mut m = HashMap::new();
        m.insert(0x3748, STLinkInfo::new("V2",    0x02,   0x81,   0x83));
        m.insert(0x374b, STLinkInfo::new("V2-1",  0x01,   0x81,   0x82));
        m.insert(0x374a, STLinkInfo::new("V2-1",  0x01,   0x81,   0x82));  // Audio
        m.insert(0x3742, STLinkInfo::new("V2-1",  0x01,   0x81,   0x82));  // No MSD
        m.insert(0x374e, STLinkInfo::new("V3",    0x01,   0x81,   0x82));
        m.insert(0x374f, STLinkInfo::new("V3",    0x01,   0x81,   0x82));  // Bridge
        m.insert(0x3753, STLinkInfo::new("V3",    0x01,   0x81,   0x82));  // 2VCP
        m
    };
}

/// A helper struct to match STLink deviceinfo.
struct STLinkInfo {
    version_name: String,
    out_ep: u8,
    in_ep: u8,
    swv_ep: u8,
}

impl STLinkInfo {
    pub fn new<V: Into<String>>(version_name: V, out_ep: u8, in_ep: u8, swv_ep: u8) -> Self {
        Self {
            version_name: version_name.into(),
            out_ep,
            in_ep,
            swv_ep,
        }
    }
}

/// Provides low-level USB enumeration and transfers for STLinkV2/3 devices.
pub struct STLinkUSBDevice<'a> {
    device: Device<'a>,
    device_handle: Option<DeviceHandle<'a>>,
    endpoint_out: u8,
    endpoint_in: u8,
    endpoint_swv: u8,
}

impl<'a> STLinkUSBDevice<'a> {
    fn usb_match(device: &Device<'a>) -> bool {
        // Check the VID/PID.
        if let Ok(descriptor) = device.device_descriptor() {
            (descriptor.vendor_id() == USB_VID)
            && (USB_PID_EP_MAP.contains_key(&descriptor.product_id()))
        } else {
            false
        }
    }

    fn get_all_plugged_devices(context: &'a Context) -> Result<Vec<STLinkUSBDevice<'a>>, Error> {
        let devices = context.devices()?;
        devices.iter()
               .filter(Self::usb_match)
               .map(|device| STLinkUSBDevice::new(device))
               .collect::<Result<Vec<_>, Error>>()
    }
    
    pub fn new(device: Device<'a>) -> Result<Self, Error> {
        let descriptor = device.device_descriptor()?;
        let info = &USB_PID_EP_MAP[&descriptor.product_id()];
        Ok(Self {
            device,
            device_handle: None,
            endpoint_out: info.out_ep,
            endpoint_in: info.in_ep,
            endpoint_swv: info.swv_ep,
        })
    }

    pub fn open(&mut self) -> Result<(), Error> {
        self.device_handle = Some(self.device.open()?);
        self.device_handle.as_mut().map(|ref mut dh| dh.claim_interface(0));

        let config = self.device.active_config_descriptor()?;
        let descriptor = self.device.device_descriptor()?;
        let info = &USB_PID_EP_MAP[&descriptor.product_id()];

        let mut endpoint_out = None;
        let mut endpoint_in = None;
        let mut endpoint_swv = None;

        if let Some(interface) = config.interfaces().next() {
            if let Some(descriptor) = interface.descriptors().next() {
                for endpoint in descriptor.endpoint_descriptors() {
                    if endpoint.address() == info.out_ep {
                        endpoint_out = Some(info.out_ep);
                    } else if endpoint.address() == info.in_ep {
                        endpoint_in = Some(info.in_ep);
                    } else if endpoint.address() == info.swv_ep {
                        endpoint_swv = Some(info.swv_ep);
                    }
                }
            }
        }
        
        if endpoint_out.is_none() {
            return Err(Error::NotFound);
            // raise STLinkException("Unable to find OUT endpoint")
        }

        if endpoint_in.is_none() {
            return Err(Error::NotFound);
            // raise STLinkException("Unable to find IN endpoint")
        }

        if endpoint_swv.is_none() {
            return Err(Error::NotFound);
            // raise STLinkException("Unable to find OUT endpoint")
        }

        self.flush_rx();

        Ok(())
    }

    pub fn close(&mut self) {
        self.device_handle.as_mut().map(|dh| dh.release_interface(0));
        self.device_handle = None;
    }

    /// Flush the RX buffers by reading until a timeout occurs.
    fn flush_rx(&mut self) {
        loop {
            if let Err(Error::Timeout) = self.read(1000, Duration::from_millis(1)) {
                break;
            }
        }
    }

    pub fn read(&mut self, size: u16, timeout: Duration) -> Result<Vec<u8>, Error> {
        let mut buf = Vec::with_capacity(size as usize);
        let ep = self.endpoint_in;
        self.device_handle.as_mut().map(|dh| dh.read_bulk(ep, buf.as_mut_slice(), timeout));
        Ok(buf)
    }

    pub fn write(&mut self, mut cmd: Vec<u8>, write_data: &[u8], read_data: &mut[u8], timeout: Duration) -> Result<(), Error> {
        // Command phase.
        for _ in 0..(CMD_LEN - cmd.len()) {
            cmd.push(0);
        }
        let ep_in = self.endpoint_in;
        let ep_out = self.endpoint_out;
        let written_bytes = self.device_handle.as_mut().map(|dh| dh.write_bulk(ep_out, &cmd, timeout)).unwrap()?;
        
        if written_bytes != CMD_LEN {
            return Err(Error::Io);
        }
        
        // Optional data out phase.
        if write_data.len() > 0 {
            let written_bytes = self.device_handle.as_mut().map(|dh| dh.write_bulk(ep_out, write_data, timeout)).unwrap()?;
            if written_bytes != write_data.len() {
                return Err(Error::Io);
            }
        }

        // Optional data in phase.
        if read_data.len() > 0 {
            let read_bytes = self.device_handle.as_mut().map(|dh| dh.read_bulk(ep_in, read_data, timeout)).unwrap()?;
            if read_bytes != read_data.len() {
                return Err(Error::Io);
            }
        }
        Ok(())
    }

    pub fn read_swv(&mut self, size: usize, timeout: Duration) -> Result<Vec<u8>, Error> {
        let mut buf = Vec::with_capacity(size as usize);
        let ep = self.endpoint_swv;
        let read_bytes = self.device_handle.as_mut().map(|dh| dh.read_bulk(ep, buf.as_mut_slice(), timeout)).unwrap()?;
        if read_bytes != size {
            return Err(Error::Io);
        } else {
            Ok(buf)
        }
    } 
}

#[test]
fn list_devices() {

}