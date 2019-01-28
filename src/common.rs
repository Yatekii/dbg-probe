pub trait BytesTo {
    fn to_u16(self) -> u16;
    fn to_u32(self) -> u32;
}

impl BytesTo for &[u8] {
    fn to_u32(self) -> u32 {
        ((self[3] as u32) << 24) | ((self[2] as u32) << 16) | ((self[1] as u32) << 8) | self[0] as u32
    }

    fn to_u16(self) -> u16 {
        ((self[1] as u16) << 8) | self[0] as u16
    }
}

#[test]
fn convert_u32() {
    use BytesTo;
    assert_eq!((&[0x42u8, 0, 0, 0][..]).to_u32(), 0x42);
    assert_eq!((&[0, 0x42u8, 0, 0][..]).to_u32(), 0x4200);
    assert_eq!((&[0, 0, 0x42u8, 0][..]).to_u32(), 0x420000);
    assert_eq!((&[0, 0, 0, 0x42u8][..]).to_u32(), 0x42000000);
}

#[test]
fn convert_u16() {
    use BytesTo;
    assert_eq!((&[0x42u8, 0][..]).to_u16(), 0x42);
    assert_eq!((&[0, 0x42u8][..]).to_u16(), 0x4200);
}