pub trait BusOperation {
    type Error;

    fn read_bytes(&mut self, rbuf: &mut [u8]) -> Result<(), Self::Error>;
    fn write_bytes(&mut self, wbuf: &[u8]) -> Result<(), Self::Error>;
    fn write_byte_read_bytes(&mut self, wbuf: &[u8; 1], rbuf: &mut [u8])
        -> Result<(), Self::Error>;
}