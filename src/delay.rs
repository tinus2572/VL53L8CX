
use crate::{BusOperation, Vl53l8cx};

impl<B: BusOperation> Vl53l8cx<B> {

    pub fn delay(&mut self, ms: u32) {
        self.delay.delay_ms(ms);
    }
}