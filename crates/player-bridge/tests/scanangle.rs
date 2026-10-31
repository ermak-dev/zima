// BUG: SCANANGLE intensities are XDR u_short elements, not packed bytes.
// HYPOTHESIS: skipping one byte per intensity reads an intensity as the scan ID.
// RESULT: FIXED — decode one XDR word per u16 intensity before the ID.
use player_bridge::decode_laser_scanangle;

fn payload(intensities: &[u32], id: u32) -> Vec<u8> {
    let mut bytes = 6.0f32.to_be_bytes().to_vec();
    for values in [&[1.0f32, 2.0][..], &[0.0f32, 0.1][..]] {
        bytes.extend((values.len() as u32).to_be_bytes());
        bytes.extend((values.len() as u32).to_be_bytes());
        for value in values {
            bytes.extend(value.to_be_bytes());
        }
    }
    bytes.extend((intensities.len() as u32).to_be_bytes());
    bytes.extend((intensities.len() as u32).to_be_bytes());
    for value in intensities {
        bytes.extend(value.to_be_bytes());
    }
    bytes.extend(id.to_be_bytes());
    bytes
}

#[test]
fn scanangle_preserves_u16_intensities_and_trailing_id() {
    let scan = decode_laser_scanangle(&payload(&[1, 32768, 65535], 1234)).unwrap();
    assert_eq!(scan.id, 1234);
    assert_eq!(
        scan.intensities
            .iter()
            .map(|&v| u32::from(v))
            .collect::<Vec<_>>(),
        vec![1, 32768, 65535]
    );
    assert_eq!(scan.ranges, vec![1.0, 2.0]);
}

#[test]
fn scanangle_rejects_truncated_intensity_or_missing_id() {
    let bytes = payload(&[32768, 400], 1234);
    for length in 44..bytes.len() {
        assert!(
            decode_laser_scanangle(&bytes[..length]).is_err(),
            "accepted length {length}"
        );
    }
    assert_eq!(decode_laser_scanangle(&payload(&[], 77)).unwrap().id, 77);
}

#[test]
fn scanangle_rejects_out_of_range_xdr_ushort() {
    assert!(decode_laser_scanangle(&payload(&[65536], 1234)).is_err());
}

// DEDUP (issue #51): single-decode moves array validation into the codec;
// a hostile count word must be rejected before any large allocation.
#[test]
fn scanangle_rejects_hostile_count_words_before_allocating() {
    // ranges_count far beyond the payload: duplicated word agrees.
    let mut bytes = 6.0f32.to_be_bytes().to_vec();
    bytes.extend(0x7fff_ffffu32.to_be_bytes());
    bytes.extend(0x7fff_ffffu32.to_be_bytes());
    assert!(decode_laser_scanangle(&bytes).is_err());
    // Same for the angles array: one valid range then a hostile angle count.
    let mut bytes = 6.0f32.to_be_bytes().to_vec();
    bytes.extend(1u32.to_be_bytes());
    bytes.extend(1u32.to_be_bytes());
    bytes.extend(1.0f32.to_be_bytes());
    bytes.extend(0xffff_ffffu32.to_be_bytes());
    bytes.extend(0xffff_ffffu32.to_be_bytes());
    assert!(decode_laser_scanangle(&bytes).is_err());
    // Mismatched duplicate counts stay a framing error.
    let mut bytes = 6.0f32.to_be_bytes().to_vec();
    bytes.extend(2u32.to_be_bytes());
    bytes.extend(3u32.to_be_bytes());
    assert!(matches!(
        decode_laser_scanangle(&bytes),
        Err(player_bridge::Error::CorruptChannel)
    ));
}

// DEDUP (issue #51): the same hostile-count guarantee for the array codecs
// that once relied on robot-server's pre-decode validation.
#[test]
fn array_codecs_reject_hostile_counts_before_allocating() {
    use player_bridge::{decode_bumper, decode_ir, decode_sonar};
    // bumper: three flags claimed, none present, count agrees.
    assert!(decode_bumper(&0x3fff_ffffu32.to_be_bytes().repeat(2)).is_err());
    // sonar: hostile range count.
    assert!(decode_sonar(&0xffff_ffffu32.to_be_bytes().repeat(2)).is_err());
    // ir: hostile voltage count.
    let mut ir = Vec::new();
    ir.extend(0x1000_0000u32.to_be_bytes());
    ir.extend(0x1000_0000u32.to_be_bytes());
    assert!(decode_ir(&ir).is_err());
}
