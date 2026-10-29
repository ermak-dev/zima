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
