pub struct ConnectedProbe<P: DebugProbe + Sized> {
    debug_probe: P,
}

impl ConnectedProbe {

}

pub enum ProbeError {
    NotConnected,
    ConnectionFailed(String)
}

pub trait DebugProbe {

    pub fn get_all_connected_probes();
    
    pub fn get_probe_with_id(unique_id: usize) -> DebugProbe;
    
    pub fn description(&self) -> String {
        self.vendor_name() + " " + self.product_name()
    }
    
    pub fn vendor_name(&self) -> String;
    
    pub fn product_name(&self) -> String;
    
    pub fn get_supported_wire_protocols(self) -> Vec<WireProtocol>;

    /// Gets the unique id of a probe.
    pub fn unique_id(&self) -> usize;

    /// Returns the currently selected `WireProtocol` if the probe is connected.
    /// Returns `None` otherwise.
    pub fn wire_protocol(&self) ->  -> Result<WireProtocol, ProbeError>;
    
    pub fn is_connected(&self) -> bool;

    pub fn connect(&self) -> Result<(), ProbeError>;
    
    pub fn close(&self);

    /// Sets the frequency for JTAG and SWD in Hz.
    pub fn set_clock(self, frequency: usize);
}