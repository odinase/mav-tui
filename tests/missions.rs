use serde_json::Value;

#[test]
fn qgc_fixtures_take_off_ten_metres_fly_twenty_metre_square_and_land_at_home() {
    for (text, firmware) in [
        (include_str!("../missions/ardupilot-square.plan"), 3),
        (include_str!("../missions/px4-square.plan"), 12),
    ] {
        let plan: Value = serde_json::from_str(text).unwrap();
        assert_eq!(plan["fileType"], "Plan");
        assert_eq!(plan["mission"]["firmwareType"], firmware);
        assert_eq!(plan["mission"]["vehicleType"], 2);
        assert_eq!(
            plan["mission"]["plannedHomePosition"],
            serde_json::json!([47.397742, 8.545594, 488])
        );
        let items = plan["mission"]["items"].as_array().unwrap();
        assert_eq!(items.len(), 6);
        assert_eq!(items[0]["command"], 22); // NAV_TAKEOFF
        assert_eq!(items[5]["command"], 21); // NAV_LAND
        for (index, item) in items.iter().enumerate() {
            assert_eq!(item["frame"], 3); // GLOBAL_RELATIVE_ALT
            assert_eq!(item["params"][6], if index == 5 { 0 } else { 10 });
            assert_eq!(item["doJumpId"], index + 1);
        }
        for index in 1..=4 {
            assert_eq!(items[index]["command"], 16);
            let from = &items[index - 1]["params"];
            let to = &items[index]["params"];
            let latitude_delta = (to[4].as_f64().unwrap() - from[4].as_f64().unwrap()).to_radians();
            let longitude_delta =
                (to[5].as_f64().unwrap() - from[5].as_f64().unwrap()).to_radians();
            let distance = 6_371_000.
                * latitude_delta.hypot(longitude_delta * 47.397742_f64.to_radians().cos());
            assert!(
                (distance - 20.).abs() < 0.1,
                "square side {index}: {distance} m"
            );
        }
        for index in [4, 5] {
            assert_eq!(items[index]["params"][4], items[0]["params"][4]);
            assert_eq!(items[index]["params"][5], items[0]["params"][5]);
        }
    }
}
