import numpy as np
from particle_filter import ParticleFilter, Robot, Config


def example_basic():
    """基本的な使用例"""
    print("=" * 60)
    print("例 1: 基本的な粒子フィルタの使用")
    print("=" * 60)
    
    # 設定
    config = Config(
        num_particles=500,
        process_noise=0.5,
        measurement_noise=3.0,
        landmark_positions=[20.0, 40.0, 60.0, 80.0]
    )
    
    # ロボットとフィルタの初期化
    true_position = 30.0
    robot = Robot(true_position, config)
    pf = ParticleFilter(config)
    
    print(f"\n初期状態:")
    print(f"  ロボットの真の位置: {robot.true_position:.2f}")
    print(f"  フィルタの推定位置: {pf.estimate_position():.2f}")
    
    # 10ステップのシミュレーション
    print(f"\n{10}ステップのシミュレーション実行:")
    for step in range(10):
        control = 3.0  # 毎回3単位前に移動
        robot.move(control)
        measurement = robot.observe_landmark()
        pf.filter_step(control, measurement)
        
        estimated = pf.estimate_position()
        error = abs(robot.true_position - estimated)
        confidence = pf.get_confidence()
        
        print(f"  Step {step+1:2d}: 推定={estimated:6.2f}, "
              f"誤差={error:5.2f}, 信頼度={confidence:5.2f}")


def example_high_noise():
    """高ノイズ環境での推定"""
    print("\n" + "=" * 60)
    print("例 2: 高ノイズ環境での推定")
    print("=" * 60)
    
    # 高ノイズ設定
    config_noisy = Config(
        num_particles=1000,
        process_noise=3.0,      # 大きなプロセスノイズ
        measurement_noise=8.0,  # 大きな計測ノイズ
    )
    
    robot = Robot(50.0, config_noisy)
    pf = ParticleFilter(config_noisy)
    
    print(f"\n初期設定:")
    print(f"  プロセスノイズ: {config_noisy.process_noise}")
    print(f"  計測ノイズ: {config_noisy.measurement_noise}")
    
    print(f"\n{15}ステップのシミュレーション:")
    for step in range(15):
        control = 2.0
        robot.move(control)
        measurement = robot.observe_landmark()
        pf.filter_step(control, measurement)
        
        estimated = pf.estimate_position()
        error = abs(robot.true_position - estimated)
        
        if step % 3 == 0:
            print(f"  Step {step+1:2d}: 推定={estimated:6.2f}, "
                  f"誤差={error:5.2f}")


def example_adaptive_behavior():
    """ロボットが加速する場合"""
    print("\n" + "=" * 60)
    print("例 3: 加速度付き移動のシミュレーション")
    print("=" * 60)
    
    config = Config(num_particles=800)
    robot = Robot(10.0, config)
    pf = ParticleFilter(config)
    
    print(f"\nロボットが徐々に加速しながら移動:")
    for step in range(20):
        # 加速度を付ける（step数に応じて移動距離を増加）
        control = 0.5 + 0.3 * step
        robot.move(control)
        measurement = robot.observe_landmark()
        pf.filter_step(control, measurement)
        
        estimated = pf.estimate_position()
        error = abs(robot.true_position - estimated)
        
        if step % 4 == 0:
            print(f"  Step {step+1:2d}: 制御={control:5.2f}, "
                  f"推定={estimated:6.2f}, 誤差={error:5.2f}")


def example_landmark_comparison():
    """ランドマーク数による影響"""
    print("\n" + "=" * 60)
    print("例 4: ランドマーク数による精度の比較")
    print("=" * 60)
    
    # ランドマークなし（困難）
    config_sparse = Config(
        num_particles=1000,
        landmark_positions=[10.0, 90.0]  # 2個のみ
    )
    
    # ランドマーク多数（容易）
    config_dense = Config(
        num_particles=1000,
        landmark_positions=[10.0, 20.0, 30.0, 40.0, 50.0, 
                           60.0, 70.0, 80.0, 90.0]  # 9個
    )
    
    for config_name, config in [("スパース（ランドマーク2個）", config_sparse),
                                 ("デンス（ランドマーク9個）", config_dense)]:
        print(f"\n{config_name}:")
        robot = Robot(45.0, config)
        pf = ParticleFilter(config)
        
        errors = []
        for step in range(30):
            control = 1.5
            robot.move(control)
            measurement = robot.observe_landmark()
            pf.filter_step(control, measurement)
            
            estimated = pf.estimate_position()
            error = abs(robot.true_position - estimated)
            errors.append(error)
        
        avg_error = np.mean(errors[-10:])  # 最後10ステップの平均誤差
        print(f"  最後10ステップの平均誤差: {avg_error:.3f}")


def example_convergence():
    """パーティクルの収束過程の観察"""
    print("\n" + "=" * 60)
    print("例 5: パーティクル分布の収束過程")
    print("=" * 60)
    
    config = Config(num_particles=500)
    robot = Robot(50.0, config)
    pf = ParticleFilter(config)
    
    # パーティクルの分散を計算する関数
    def particle_variance():
        weighted_mean = np.average(pf.particles, weights=pf.weights)
        variance = np.average((pf.particles - weighted_mean) ** 2, 
                             weights=pf.weights)
        return variance
    
    print(f"\nシミュレーション進行に伴うパーティクル分布の収束:")
    print(f"  初期分散: {particle_variance():.2f}")
    
    for step in range(25):
        control = 2.0
        robot.move(control)
        measurement = robot.observe_landmark()
        pf.filter_step(control, measurement)
        
        if step % 5 == 4:
            variance = particle_variance()
            estimated = pf.estimate_position()
            print(f"  Step {step+1:2d}: 分散={variance:8.2f}, "
                  f"推定位置={estimated:6.2f}")


if __name__ == '__main__':
    print("\n")
    print("╔" + "=" * 58 + "╗")
    print("║" + " " * 58 + "║")
    print("║" + "  パーティクルフィルタの利用例".center(58) + "║")
    print("║" + " " * 58 + "║")
    print("╚" + "=" * 58 + "╝")
    print()
    
    example_basic()
    example_high_noise()
    example_adaptive_behavior()
    example_landmark_comparison()
    example_convergence()
    
    print("\n" + "=" * 60)
    print("全ての例が完了しました！")
    print("=" * 60)
