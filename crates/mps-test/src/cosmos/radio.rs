// ---------------------------------------------------------------------------
// 单元测试
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use mps_cosmos::radio::*;
    use rapier3d::prelude::Vector;

    /// 一条波按 100 AU/s 飞（默认），几十公里这种距离出生后 1 ms 就已经扫过了：
    /// 波前已经扫过测试里那些几千公里的距离，但还没到噪声底。
    const STEP_MS: u64 = 1;

    fn node(id: u64, pos: Vector) -> RadioNode {
        RadioNode {
            id,
            pos,
            vel: Vector::ZERO,
            dir: Vector::new(0.0, 0.0, 1.0),
            frequency: 2.4e9,
            power: 50.0,
            sensitivity: 1.0e-15,
            rx_gain: 1.0,
            tx_gain: 1.0,
            beam_angle: std::f64::consts::PI,
            rx_beam_angle: std::f64::consts::PI,
            owner_body: None,
        }
    }

    fn signal(tx_id: u64, tx_pos: Vector) -> ActiveSignal {
        ActiveSignal {
            id: tx_id * 100 + 1,
            birth_ms: 0,
            origin: tx_pos,
            origin_vel: Vector::ZERO,
            origin_dir: Vector::new(1.0, 0.0, 0.0),
            frequency: 2.4e9,
            energy: 50.0,
            tx_gain: 1.0,
            beam_angle: std::f64::consts::PI,
            owner_body: None,
            schedule: Vec::new(),
            cursor: 0,
        }
    }

    fn reflector(handle: u64, pos: Vector, radius: f64) -> ReflectorState {
        ReflectorState {
            handle_raw: handle,
            pos,
            radius,
        }
    }

    #[test]
    fn direct_line_of_sight_reaches() {
        let mut radio = RadioWorld::new();
        radio.register_node(node(1, Vector::new(0.0, 0.0, 0.0)));
        radio.register_node(node(2, Vector::new(1000.0, 0.0, 0.0)));
        radio.submit_signal(ActiveSignal {
            id: 1,
            birth_ms: 0,
            origin: Vector::new(0.0, 0.0, 0.0),
            origin_vel: Vector::ZERO,
            origin_dir: Vector::new(1.0, 0.0, 0.0),
            frequency: 2.4e9,
            energy: 50.0,
            tx_gain: 1.0,
            beam_angle: std::f64::consts::PI,
            owner_body: None,
            schedule: Vec::new(),
            cursor: 0,
        });

        // 无天体遮挡 → 直达
        radio.step(&[], STEP_MS);
        let results = radio.take_results();
        assert_eq!(results.len(), 1);
        assert_eq!(results[0].rx_node_id, 2);
        assert!(results[0].received_power > 0.0);
    }

    #[test]
    fn celestial_body_blocks_direct_path_but_third_body_reflects() {
        // 场景：大行星挡在 A→B 直线上（直接不可达），
        // 但 A、B 都能"看到"旁边的小反射体 M，经 M 一次反射应可达。
        let mut radio = RadioWorld::new();
        // A 与 B 在大行星两侧（行星半径 4000，A/B 距原点 8000）→ 直线穿过行星被挡
        radio.register_node(node(1, Vector::new(-8000.0, 0.0, 0.0)));
        radio.register_node(node(2, Vector::new(8000.0, 0.0, 0.0)));
        radio.submit_signal(ActiveSignal {
            id: 1,
            birth_ms: 0,
            origin: Vector::new(-8000.0, 0.0, 0.0),
            origin_vel: Vector::ZERO,
            origin_dir: Vector::new(1.0, 0.0, 0.0),
            frequency: 2.4e9,
            energy: 50.0,
            tx_gain: 1.0,
            beam_angle: std::f64::consts::PI,
            owner_body: None,
            schedule: Vec::new(),
            cursor: 0,
        });

        // 挡路大行星在原点；反射体 M 在侧面高处，A/B 均可见
        let bodies = vec![
            ReflectorState {
                handle_raw: 1,
                pos: Vector::ZERO,
                radius: 4000.0,
            },
            ReflectorState {
                handle_raw: 2,
                pos: Vector::new(0.0, 30000.0, 0.0),
                radius: 1500.0,
            },
        ];
        radio.step(&bodies, STEP_MS);
        let results = radio.take_results();
        // 反射体在 y=20000，A/B 视线均不穿过它 → 反射路径应可达
        assert!(
            !results.is_empty(),
            "expected a reflected path to reach receiver, got none"
        );
        assert!(results[0].received_power > 0.0);
    }

    #[test]
    fn planet_blocks_direct_path_no_reflector_unreachable() {
        // 挡路行星在中间，无反射体 → 必须不可达
        let mut radio = RadioWorld::new();
        radio.register_node(node(1, Vector::new(-8000.0, 0.0, 0.0)));
        radio.register_node(node(2, Vector::new(8000.0, 0.0, 0.0)));
        radio.submit_signal(signal(1, Vector::new(-8000.0, 0.0, 0.0)));

        let bodies = vec![reflector(1, Vector::ZERO, 4000.0)];
        radio.step(&bodies, STEP_MS);
        let results = radio.take_results();
        assert!(results.is_empty(), "blocked path must not reach");
    }

    #[test]
    fn sensitivity_filters_weak_signal() {
        let mut radio = RadioWorld::new();
        // 1000 米、50W：1/d² = 1e-6 → 5e-5 W，超过默认灵敏度 1e-15
        // 但这里把灵敏度设成 1.0 W → 应被滤掉
        let mut rx = node(2, Vector::new(1000.0, 0.0, 0.0));
        rx.sensitivity = 1.0;
        radio.register_node(node(1, Vector::ZERO));
        radio.register_node(rx);
        radio.submit_signal(signal(1, Vector::ZERO));

        radio.step(&[], STEP_MS);
        assert!(
            radio.take_results().is_empty(),
            "weak signal must be filtered"
        );
    }

    #[test]
    fn directional_beam_excludes_out_of_cone() {
        let mut radio = RadioWorld::new();
        let mut tx = node(1, Vector::ZERO);
        tx.tx_gain = 10.0;
        tx.beam_angle = 0.1; // 约 5.7°，朝 +X
        radio.register_node(tx);
        // 接收端在 +Y 方向，明显在波束外
        radio.register_node(node(2, Vector::new(0.0, 1000.0, 0.0)));
        let mut s = signal(1, Vector::ZERO);
        s.tx_gain = 10.0;
        s.beam_angle = 0.1;
        radio.submit_signal(s);

        radio.step(&[], STEP_MS);
        assert!(
            radio.take_results().is_empty(),
            "out-of-cone must not reach"
        );
    }

    #[test]
    fn doppler_shift_moves_frequency() {
        let mut radio = RadioWorld::new();
        radio.register_node(node(1, Vector::ZERO));
        let mut rx = node(2, Vector::new(1.0e7, 0.0, 0.0));
        rx.vel = Vector::new(3000.0, 0.0, 0.0); // 朝发射端飞
        radio.register_node(rx);
        radio.submit_signal(signal(1, Vector::ZERO));

        radio.step(&[], STEP_MS);
        let results = radio.take_results();
        assert_eq!(results.len(), 1);
        // v_rel = 3000（向源运动为正→频率升高？源静止，接收端向源：v_rel = -3000）
        // 公式：f * (1 + v_rel/c)。接收端沿 -X 运动 → v_rel = -3000 → 频率下降
        let f = results[0].received_frequency;
        assert!((f - 2.4e9).abs() > 1.0, "frequency should shift, got {f}");
    }

    #[test]
    fn wave_reaches_near_receiver_before_far_one() {
        // 波前按波速往外扩：近端先收到、"有人收到了"也不会让波消失，远端随后还能收到
        let mut radio = RadioWorld::new();
        radio.register_node(node(1, Vector::ZERO));
        radio.register_node(node(2, Vector::new(1.0e11, 0.0, 0.0))); // 近端 0.67 AU
        radio.register_node(node(3, Vector::new(3.0e11, 0.0, 0.0))); // 远端 2 AU
        let mut s = signal(1, Vector::ZERO);
        s.energy = 1.0e10; // 远端在 2 AU 外，能量得给够才判得过灵敏度
        radio.submit_signal(s);

        // t=0：波前还在原点
        radio.step(&[], 0);
        assert!(radio.take_results().is_empty());

        // t=10ms：波前 1.5e11 m（100 AU/s）→ 只有近端
        radio.step(&[], 10);
        let near = radio.take_results();
        assert_eq!(near.len(), 1);
        assert_eq!(near[0].rx_node_id, 2);

        // t=25ms：波前 3.7e11 m → 远端也到了
        radio.step(&[], 25);
        let far = radio.take_results();
        assert_eq!(far.len(), 1);
        assert_eq!(far[0].rx_node_id, 3);
        assert_eq!(radio.signal_count(), 1, "有人收到不代表波销毁");

        // 再走一步：两端都不该重复收
        radio.step(&[], 30);
        assert!(radio.take_results().is_empty());
    }

    #[test]
    fn wave_delivers_each_node_once() {
        let mut radio = RadioWorld::new();
        radio.register_node(node(1, Vector::ZERO));
        radio.register_node(node(2, Vector::new(1000.0, 0.0, 0.0)));
        radio.submit_signal(signal(1, Vector::ZERO));

        // 波前扫过它那一步投递
        radio.step(&[], STEP_MS);
        assert_eq!(radio.take_results().len(), 1);

        // 后面不再重复投递
        radio.step(&[], STEP_MS * 2);
        assert!(radio.take_results().is_empty());
    }

    #[test]
    fn wave_dies_below_noise_floor() {
        // 强度掉到背景噪声以下就销毁：射程 = sqrt(能量 / 噪声底)
        let mut radio = RadioWorld::new();
        radio.register_node(node(1, Vector::ZERO)); // 只有发射端，没人收
        radio.submit_signal(signal(1, Vector::ZERO));

        let range = max_range(50.0);
        let inside_ms = (range * 0.5 / WAVE_SPEED * 1000.0) as u64;
        radio.step(&[], inside_ms);
        assert_eq!(radio.signal_count(), 1, "射程内应该还在");

        let outside_ms = (range * 1.001 / WAVE_SPEED * 1000.0) as u64 + 1;
        radio.step(&[], outside_ms);
        assert_eq!(radio.signal_count(), 0, "过了射程就该销毁");
        assert!(radio.take_results().is_empty());
    }

    #[test]
    fn receiver_beam_rejects_signal_from_behind() {
        // 接收天线朝 +X，信号从 -X 来（背后）→ 接收锥外，不收
        let mut radio = RadioWorld::new();
        radio.register_node(node(1, Vector::new(-1000.0, 0.0, 0.0)));
        let mut rx = node(2, Vector::ZERO);
        rx.dir = Vector::new(1.0, 0.0, 0.0);
        rx.rx_beam_angle = 0.2; // 约 11° 的接收锥
        radio.register_node(rx);
        radio.submit_signal(signal(1, Vector::new(-1000.0, 0.0, 0.0)));

        radio.step(&[], STEP_MS);
        assert!(radio.take_results().is_empty(), "背后的信号不能收");
    }

    #[test]
    fn receiver_beam_accepts_signal_from_front() {
        let mut radio = RadioWorld::new();
        radio.register_node(node(1, Vector::new(-1000.0, 0.0, 0.0)));
        let mut rx = node(2, Vector::ZERO);
        rx.dir = Vector::new(-1.0, 0.0, 0.0); // 朝着来波方向
        rx.rx_beam_angle = 0.2;
        radio.register_node(rx);
        radio.submit_signal(signal(1, Vector::new(-1000.0, 0.0, 0.0)));

        radio.step(&[], STEP_MS);
        let results = radio.take_results();
        assert_eq!(results.len(), 1);
        assert_eq!(results[0].rx_node_id, 2);
    }

    #[test]
    fn receiver_gain_tapers_with_angle() {
        // 接收天线朝 -X，接收锥 1 rad，系数 4：正对满增益、偏了变低、出锥 0
        let mut rx = node(2, Vector::ZERO);
        rx.dir = Vector::new(-1.0, 0.0, 0.0);
        rx.rx_beam_angle = 1.0;
        rx.rx_gain = 4.0;

        let on = rx.rx_gain_factor(Vector::new(-100.0, 0.0, 0.0));
        assert!((on - 4.0).abs() < 1e-9, "正对该是满系数，实际 {on}");
        // 偏 45°（0.785 rad，锥内）
        let mid = rx.rx_gain_factor(Vector::new(-100.0, 100.0, 0.0));
        assert!(mid > 0.0 && mid < 4.0, "偏 45° 该掉下来，实际 {mid}");
        // 正后方（超出锥）
        assert_eq!(
            rx.rx_gain_factor(Vector::new(100.0, 0.0, 0.0)),
            0.0,
            "锥外必须 0"
        );

        // 全向天线只有系数
        let mut omni = node(3, Vector::ZERO);
        omni.rx_gain = 2.5;
        assert!((omni.rx_gain_factor(Vector::new(100.0, 0.0, 0.0)) - 2.5).abs() < 1e-9);
    }

    #[test]
    fn receiver_gain_scales_received_power() {
        // 同样距离、同样发射端：正对的接收端收到的功率应该明显高于偏 45° 的那个
        let mut radio = RadioWorld::new();
        radio.register_node(node(1, Vector::ZERO));

        let mut on = node(2, Vector::new(1.0e5, 0.0, 0.0));
        on.dir = Vector::new(-1.0, 0.0, 0.0);
        on.rx_beam_angle = 1.0;
        on.rx_gain = 4.0;
        radio.register_node(on);

        // 距离同样是 1e5（45° 方向上），只是没正对
        let mut off = node(3, Vector::new(70710.678, 70710.678, 0.0));
        off.dir = Vector::new(-1.0, 0.0, 0.0);
        off.rx_beam_angle = 1.0;
        off.rx_gain = 4.0;
        radio.register_node(off);

        radio.submit_signal(signal(1, Vector::ZERO));
        radio.step(&[], STEP_MS);
        let results = radio.take_results();
        assert_eq!(results.len(), 2, "两个都该在锥内收到");

        let on_power = results
            .iter()
            .find(|r| r.rx_node_id == 2)
            .unwrap()
            .received_power;
        let off_power = results
            .iter()
            .find(|r| r.rx_node_id == 3)
            .unwrap()
            .received_power;
        assert!(
            on_power > off_power * 2.0,
            "正对该比偏 45° 强得多：{on_power} vs {off_power}"
        );
    }

    /// 偏出视线的天体（球心离视线 2.5 倍半径）不该挡直射。
    #[test]
    fn planet_off_the_line_does_not_block() {
        let mut radio = RadioWorld::new();
        radio.register_node(node(1, Vector::new(-8000.0, 0.0, 0.0)));
        radio.register_node(node(2, Vector::new(8000.0, 0.0, 0.0)));
        radio.submit_signal(signal(1, Vector::new(-8000.0, 0.0, 0.0)));

        let bodies = vec![reflector(1, Vector::new(0.0, 10000.0, 0.0), 4000.0)];
        radio.step(&bodies, STEP_MS);
        let results = radio.take_results();
        assert_eq!(results.len(), 1, "不在视线上就不该判成遮挡");
        assert_eq!(results[0].rx_node_id, 2);
    }

    /// 目标后面的天体不该挡（射线只在 A→B 段内找遮挡）。
    #[test]
    fn planet_behind_receiver_does_not_block() {
        let mut radio = RadioWorld::new();
        radio.register_node(node(1, Vector::new(-8000.0, 0.0, 0.0)));
        radio.register_node(node(2, Vector::new(8000.0, 0.0, 0.0)));
        radio.submit_signal(signal(1, Vector::new(-8000.0, 0.0, 0.0)));

        let bodies = vec![reflector(1, Vector::new(30000.0, 0.0, 0.0), 4000.0)];
        radio.step(&bodies, STEP_MS);
        assert_eq!(radio.take_results().len(), 1, "接收端后面的天体不该算遮挡");
    }

    /// 发射端后面的天体不该挡。
    #[test]
    fn planet_behind_source_does_not_block() {
        let mut radio = RadioWorld::new();
        radio.register_node(node(1, Vector::new(-8000.0, 0.0, 0.0)));
        radio.register_node(node(2, Vector::new(8000.0, 0.0, 0.0)));
        radio.submit_signal(signal(1, Vector::new(-8000.0, 0.0, 0.0)));

        let bodies = vec![reflector(1, Vector::new(-30000.0, 0.0, 0.0), 4000.0)];
        radio.step(&bodies, STEP_MS);
        assert_eq!(radio.take_results().len(), 1, "发射端后面的天体不该算遮挡");
    }

    /// 天线贴在行星表面时，它自己那颗行星不能挡住它（否则地面上根本发不出去）。
    #[test]
    fn own_planet_does_not_block() {
        let mut radio = RadioWorld::new();
        // 接收端就在半径 4000 的球面上，发射端在球外上方
        radio.register_node(node(1, Vector::new(0.0, 10000.0, 0.0)));
        radio.register_node(node(2, Vector::new(4000.0, 0.0, 0.0)));
        radio.submit_signal(signal(1, Vector::new(0.0, 10000.0, 0.0)));

        let bodies = vec![reflector(1, Vector::ZERO, 4000.0)];
        radio.step(&bodies, STEP_MS);
        let results = radio.take_results();
        assert_eq!(results.len(), 1, "不该被自己所在的行星挡掉");
        assert_eq!(results[0].rx_node_id, 2);
    }

    /// 反射：路径更长又乘 0.8 衰减，收到的功率该明显低于直射，但仍然是有效投递。
    #[test]
    fn reflection_is_weaker_than_direct_but_delivers() {
        let power_of = |blocked: bool| -> f64 {
            let mut radio = RadioWorld::new();
            radio.register_node(node(1, Vector::new(-8000.0, 0.0, 0.0)));
            radio.register_node(node(2, Vector::new(8000.0, 0.0, 0.0)));
            radio.submit_signal(signal(1, Vector::new(-8000.0, 0.0, 0.0)));
            let bodies = if blocked {
                vec![
                    reflector(1, Vector::ZERO, 4000.0),
                    reflector(2, Vector::new(0.0, 30000.0, 0.0), 1500.0),
                ]
            } else {
                Vec::new()
            };
            radio.step(&bodies, STEP_MS);
            radio
                .take_results()
                .iter()
                .map(|r| r.received_power)
                .fold(0.0, f64::max)
        };

        let direct = power_of(false);
        let reflected = power_of(true);
        assert!(direct > 0.0, "直射该收到");
        assert!(reflected > 0.0, "被挡之后该靠反射收到");
        assert!(
            reflected < direct * 0.1,
            "反射该明显更弱：reflected={reflected} direct={direct}"
        );
    }

    /// 接收端在动：时刻表要按"波前追上接收端"解，不是拿发射那一刻的距离除波速。
    #[test]
    fn schedule_accounts_for_receiver_motion() {
        // 波速 1e11 m/s、接收端在 1e11 m 外：
        // 静止 → 1 s 到；以半波速背离飞 → 要追 2 s
        let run = |away_speed: f64| -> (bool, bool) {
            let mut radio = RadioWorld::new();
            radio.set_wave_speed(1.0e11);
            let mut tx = node(1, Vector::ZERO);
            tx.power = 1.0e10;
            radio.register_node(tx);
            let mut rx = node(2, Vector::new(1.0e11, 0.0, 0.0));
            rx.vel = Vector::new(away_speed, 0.0, 0.0); // 背离发射端
            radio.register_node(rx);
            let mut s = signal(1, Vector::ZERO);
            s.energy = 1.0e10;
            radio.submit_signal(s);

            radio.step(&[], 1000);
            let at_1s = !radio.take_results().is_empty();
            radio.step(&[], 2100);
            let at_2s = !radio.take_results().is_empty();
            (at_1s, at_2s)
        };

        assert_eq!(run(0.0), (true, false), "静止的接收端 1 s 到");
        assert_eq!(
            run(5.0e10),
            (false, true),
            "以半波速背离飞，要追到 2 s 才到"
        );
    }

    /// 反射预筛不能误杀"刚刚够门限"的反射：它用的是路径下界，比真实路径只会更严。
    #[test]
    fn weak_reflection_survives_prefilter() {
        let hits = |energy: f64| -> usize {
            let mut radio = RadioWorld::new();
            radio.register_node(node(1, Vector::new(-8000.0, 0.0, 0.0)));
            radio.register_node(node(2, Vector::new(8000.0, 0.0, 0.0)));
            let mut s = signal(1, Vector::new(-8000.0, 0.0, 0.0));
            s.energy = energy;
            radio.submit_signal(s);
            let bodies = vec![
                reflector(1, Vector::ZERO, 4000.0),
                reflector(2, Vector::new(0.0, 30000.0, 0.0), 1500.0),
            ];
            radio.step(&bodies, STEP_MS);
            radio.take_results().len()
        };

        assert_eq!(hits(2.0e-5), 1, "刚过门限的反射不能被预筛误杀");
        assert_eq!(hits(1.0e-6), 0, "低于门限的反射该被丢掉");
    }

    /// 定向发射时，波束扫不到的天体不该被当成反射面：全向能反射，30° 锥就反射不了。
    #[test]
    fn reflection_respects_tx_beam() {
        let hits = |directive: bool| -> usize {
            let mut radio = RadioWorld::new();
            let mut tx = node(1, Vector::new(-8000.0, 0.0, 0.0));
            let mut s = signal(1, Vector::new(-8000.0, 0.0, 0.0));
            if directive {
                tx.tx_gain = 10.0;
                tx.beam_angle = (30f64).to_radians();
                s.tx_gain = 10.0;
                s.beam_angle = (30f64).to_radians();
            }
            radio.register_node(tx);
            radio.register_node(node(2, Vector::new(8000.0, 0.0, 0.0)));
            radio.submit_signal(s);
            // 挡路的在轴上（锥内）；反射体在 75° 方向（30° 锥外，波扫不到）
            let bodies = vec![
                reflector(1, Vector::ZERO, 4000.0),
                reflector(2, Vector::new(0.0, 30000.0, 0.0), 1500.0),
            ];
            radio.step(&bodies, STEP_MS);
            radio.take_results().len()
        };

        assert_eq!(hits(false), 1, "全向发射：被挡之后能靠反射收到");
        assert_eq!(hits(true), 0, "定向 30°：反射体在波束外，波扫不到它");
    }

    /// 基准（不进常规测试）：
    /// `cargo test --release -p mps-cosmos -- --ignored --nocapture radio_bench`
    #[test]
    #[ignore = "手动跑的基准"]
    fn radio_bench_100_tx_100_rx() {
        use std::time::Instant;

        const TX: u64 = 100;
        const RX: u64 = 100;
        const RUNS: u32 = 10;

        // 0=无天体 1=3 个天体都在收发之间 2=定向 30°，候选天体在锥外 3=候选天体在 1e13 m（反射太弱）
        let cases: [&str; 4] = [
            "无天体（全直达）",
            "3 个天体都挡在中间",
            "定向 30°：候选天体在锥外",
            "候选天体在 1e13 m（反射太弱）",
        ];

        for (case, label) in cases.iter().enumerate() {
            let mut total_ms = 0.0f64;
            let mut idle_ms = 0.0f64;
            let mut submit_ms = 0.0f64;
            let mut results = 0usize;
            for _ in 0..RUNS {
                let mut radio = RadioWorld::new();
                for i in 0..TX {
                    let mut tx = node(i + 1, Vector::new(i as f64 * 1.0e6, 0.0, 0.0));
                    tx.power = 1.0e10;
                    if case == 2 {
                        tx.tx_gain = 10.0;
                        tx.beam_angle = (30f64).to_radians();
                    }
                    radio.register_node(tx);
                }
                for j in 0..RX {
                    // 都贴着 +X 轴排（定向场景下也在 30° 锥内）
                    radio.register_node(node(j + 1000, Vector::new(j as f64 * 1.0e6, 1.0e5, 0.0)));
                }
                // 发射（含解析到达时刻表）的开销
                let submit_start = Instant::now();
                for i in 0..TX {
                    let mut s = signal(i + 1, Vector::new(i as f64 * 1.0e6, 0.0, 0.0));
                    s.id = i + 1;
                    s.energy = 1.0e10;
                    if case == 2 {
                        s.tx_gain = 10.0;
                        s.beam_angle = (30f64).to_radians();
                    }
                    radio.submit_signal(s);
                }
                submit_ms += submit_start.elapsed().as_secs_f64() * 1000.0;

                let bodies: Vec<ReflectorState> = match case {
                    0 => Vec::new(),
                    1 => (0..3)
                        .map(|k| {
                            reflector(
                                k + 1,
                                Vector::new(5.0e7 + k as f64 * 2.0e7, 5.0e6, 0.0),
                                2.0e7,
                            )
                        })
                        .collect(),
                    2 => vec![
                        // 轴上这个挡路（在 30° 锥内），另外两个偏 45°（锥外，应该被预筛掉）
                        reflector(1, Vector::new(5.0e7, 0.0, 0.0), 2.0e7),
                        reflector(2, Vector::new(5.0e7, 5.0e7, 0.0), 2.0e7),
                        reflector(3, Vector::new(7.0e7, -5.0e7, 0.0), 2.0e7),
                    ],
                    _ => vec![
                        reflector(1, Vector::new(5.0e7, 0.0, 0.0), 2.0e7),
                        reflector(2, Vector::new(1.0e13, 0.0, 0.0), 2.0e7),
                        reflector(3, Vector::new(-1.0e13, 0.0, 0.0), 2.0e7),
                    ],
                };

                // 一步就够了：波前 1 ms 已经跑到 1.5e10 m，全部接收端都被扫到
                let start = Instant::now();
                radio.step(&bodies, 1);
                total_ms += start.elapsed().as_secs_f64() * 1000.0;
                results = radio.take_results().len();

                // 第二步：该扫的都扫完了，只剩"波还活着"的每步维护开销
                let start = Instant::now();
                radio.step(&bodies, 2);
                idle_ms += start.elapsed().as_secs_f64() * 1000.0;
            }
            println!(
                "{label}：{TX} 发 × {RX} 收 → 发射建表 {:.3} ms，首次判定 {:.3} ms（{} 条结果），之后每步 {:.3} ms",
                submit_ms / RUNS as f64,
                total_ms / RUNS as f64,
                results,
                idle_ms / RUNS as f64,
            );
        }
    }

    #[test]
    fn wave_speed_is_settable() {
        // 默认是 10 AU/s；非法值回落默认；改成别的值真的改变到达时间
        let mut radio = RadioWorld::new();
        assert!((radio.wave_speed() - WAVE_SPEED).abs() < 1.0);
        radio.set_wave_speed(1.0e12);
        assert!((radio.wave_speed() - 1.0e12).abs() < 1.0);
        radio.set_wave_speed(f64::NAN);
        assert!(
            (radio.wave_speed() - WAVE_SPEED).abs() < 1.0,
            "非法值回落默认"
        );

        // 1e11 m 外：波速 1e11 m/s 时 1 s 才到
        let mut slow = RadioWorld::new();
        slow.register_node(node(1, Vector::ZERO));
        slow.register_node(node(2, Vector::new(1.0e11, 0.0, 0.0)));
        let mut s = signal(1, Vector::ZERO);
        s.energy = 1.0e10;
        slow.submit_signal(s);
        slow.set_wave_speed(1.0e11);

        slow.step(&[], 100); // 波前才 1e10 m
        assert!(slow.take_results().is_empty(), "波速慢了就不该到");
        slow.step(&[], 1200); // 波前 1.2e11 m
        assert_eq!(slow.take_results().len(), 1);
    }
    // ── 以下 3 个用例自上游迁入（origin/main 的 mps-test radio 镜像），
    //    已适配本地 radio API：无 tx_node_id、RadioNode 带 rx_beam_angle、
    //    波前传播模型（需要推进 STEP_MS 才可能投递）、无固定 TTL 常量。 ──

    #[test]
    fn registers_and_unregisters_nodes() {
        let mut world = RadioWorld::new();
        assert_eq!(world.node_count(), 0);
        assert_eq!(world.signal_count(), 0);
        assert_eq!(world.reflector_count(), 0);

        world.register_node(node(1, Vector::ZERO));
        world.register_node(node(2, Vector::new(100.0, 0.0, 0.0)));
        assert_eq!(world.node_count(), 2);

        // 同 id 重注册覆盖而不是重复。
        world.register_node(node(1, Vector::new(5.0, 0.0, 0.0)));
        assert_eq!(world.node_count(), 2);

        world.unregister_node(1);
        assert_eq!(world.node_count(), 1);
        assert_eq!(world.node_ids().collect::<Vec<_>>(), vec![2]);
    }

    #[test]
    fn delivers_signal_on_clear_line_of_sight() {
        let mut world = RadioWorld::new();
        world.register_node(node(1, Vector::ZERO));
        world.register_node(node(2, Vector::new(1000.0, 0.0, 0.0)));
        world.submit_signal(signal(1, Vector::ZERO));

        world.step(&[], STEP_MS);
        let results = world.take_results();
        assert_eq!(results.len(), 1);
        // `signal(tx_id, pos)` 生成的 id 是 `tx_id * 100 + 1`。
        assert_eq!(results[0].signal_id, 101);
        assert_eq!(results[0].rx_node_id, 2);
        assert!((results[0].received_frequency - 2.4e9).abs() < 1.0e-6);
        assert!(results[0].received_power > 0.0);
        // 本地模型：时刻表处理完不算结束，信号在后续步的波前衰减清理中离开活跃池。
        world.step(&[], 1_000_000);
        assert!(world.take_results().is_empty());
        assert_eq!(world.signal_count(), 0);
    }

    #[test]
    fn reflector_between_nodes_blocks_direct_path() {
        let mut world = RadioWorld::new();
        world.register_node(node(1, Vector::ZERO));
        world.register_node(node(2, Vector::new(1000.0, 0.0, 0.0)));
        // 一个行星大小的反射体正好挡在两个节点之间。
        let bodies = [ReflectorState {
            handle_raw: 10,
            pos: Vector::new(500.0, 0.0, 0.0),
            radius: 100.0,
        }];
        world.submit_signal(signal(1, Vector::ZERO));

        world.step(&bodies, STEP_MS);
        // 没有第三个天体可反射 → 本轮不投递，信号留在活跃池。
        assert!(world.take_results().is_empty());
        assert_eq!(world.signal_count(), 1);

        // 波前衰减到噪声底之后，信号被清理。
        world.step(&bodies, 1_000_000);
        assert!(world.take_results().is_empty());
        assert_eq!(world.signal_count(), 0);
    }
}
