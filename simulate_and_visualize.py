import numpy as np
import matplotlib
matplotlib.use('Agg')  # 非対話バックエンド
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from particle_filter import ParticleFilter, Robot, Config


def simulate(num_steps: int = 50, seed: int = 42) -> tuple:
    """
    シミュレーションを実行
    
    Args:
        num_steps: シミュレーションステップ数
        seed: 乱数シード
        
    Returns:
        (robot, particle_filter): ロボットとフィルタのインスタンス
    """
    np.random.seed(seed)
    
    # 設定
    config = Config(
        num_particles=1000,
        process_noise=0.5,
        measurement_noise=3.0
    )
    
    # ロボットとフィルタの初期化
    true_position = 25.0
    robot = Robot(true_position, config)
    pf = ParticleFilter(config)
    
    # シミュレーション実行
    for step in range(num_steps):
        # 制御入力（毎ステップ2単位移動）
        control = 2.0
        
        # ロボットを動かす
        robot.move(control)
        
        # センサー観測
        measurement = robot.observe_landmark()
        
        # フィルタ処理
        pf.filter_step(control, measurement)
        
        # 進捗出力
        estimated_pos = pf.estimate_position()
        error = abs(robot.true_position - estimated_pos)
        print(f"Step {step + 1}: True={robot.true_position:.2f}, "
              f"Estimated={estimated_pos:.2f}, Error={error:.2f}")
    
    return robot, pf, config


def plot_results(robot: Robot, pf: ParticleFilter, config: Config):
    """
    シミュレーション結果をプロット
    
    Args:
        robot: ロボットインスタンス
        pf: パーティクルフィルタインスタンス
        config: シミュレーション設定
    """
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    
    # 1. 位置推定の精度
    ax = axes[0]
    estimated_positions = [
        np.average(particles, weights=weights)
        for particles, weights in zip(pf.particle_history, pf.weight_history)
    ]
    steps = range(len(robot.position_history))
    
    ax.plot(steps, robot.position_history, 'g-', linewidth=2.5, label='True Position')
    ax.plot(steps, estimated_positions, 'r--', linewidth=2, label='Estimated Position')
    ax.scatter(config.landmark_positions, [-5] * len(config.landmark_positions),
              marker='v', s=100, c='blue', label='Landmarks', zorder=5)
    ax.set_xlabel('Time Step')
    ax.set_ylabel('Position')
    ax.set_ylim(-10, config.world_size)
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_title('Position Tracking')
    
    # 2. パーティクルの分布（複数時刻）
    ax = axes[1]
    time_indices = [0, len(pf.particle_history) // 3, 
                   2 * len(pf.particle_history) // 3, -1]
    colors = ['red', 'orange', 'purple', 'green']
    labels = [f't={i}' if i >= 0 else f't=final' for i in time_indices]
    
    for idx, (time_idx, color, label) in enumerate(zip(time_indices, colors, labels)):
        particles = pf.particle_history[time_idx]
        weights = pf.weight_history[time_idx]
        # アルファを調整して見やすく
        alpha = 0.3 + 0.2 * idx
        ax.scatter(particles, [time_idx] * len(particles), 
                  alpha=alpha, s=10, c=color, label=label)
    
    ax.scatter(config.landmark_positions, [-0.5] * len(config.landmark_positions),
              marker='v', s=100, c='blue', label='Landmarks', zorder=5)
    ax.set_xlabel('Position')
    ax.set_ylabel('Time Index')
    ax.set_xlim(-5, config.world_size + 5)
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_title('Particle Distribution Over Time')
    
    # 3. 推定誤差
    ax = axes[2]
    errors = []
    for particles, weights in zip(pf.particle_history, pf.weight_history):
        estimated = np.average(particles, weights=weights)
        true_val = robot.position_history[len(errors)]
        error = abs(true_val - estimated)
        errors.append(error)
    
    ax.plot(errors, 'b-', linewidth=2, label='Estimation Error')
    ax.fill_between(range(len(errors)), errors, alpha=0.3)
    ax.set_xlabel('Time Step')
    ax.set_ylabel('Absolute Error')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_title('Estimation Error Over Time')
    
    plt.tight_layout()
    plt.savefig('simulation_results.png', dpi=150, bbox_inches='tight')
    print("\n✓ 結果を 'simulation_results.png' に保存しました")
    plt.close(fig)


def plot_particle_animation(robot: Robot, pf: ParticleFilter, config: Config):
    """
    パーティクルフィルタのアニメーション
    
    Args:
        robot: ロボットインスタンス
        pf: パーティクルフィルタインスタンス
        config: シミュレーション設定
    """
    fig, ax = plt.subplots(figsize=(12, 4))
    
    def animate(frame):
        ax.clear()
        
        particles = pf.particle_history[frame]
        weights = pf.weight_history[frame]
        true_pos = robot.position_history[frame]
        estimated_pos = np.average(particles, weights=weights)
        
        # パーティクルをプロット
        sizes = weights * 1000 + 10  # 重みに応じてサイズを変更
        ax.scatter(particles, [1] * len(particles), s=sizes, 
                  alpha=0.6, c='red', label='Particles')
        
        # ランドマーク
        ax.scatter(config.landmark_positions, [1] * len(config.landmark_positions),
                  marker='v', s=100, c='blue', label='Landmarks', zorder=5)
        
        # 真の位置
        ax.scatter([true_pos], [1.05], marker='*', s=300, c='green', 
                  label='True Position', zorder=4)
        
        # 推定位置
        ax.scatter([estimated_pos], [0.95], marker='s', s=100, c='orange', 
                  label='Estimated Position', zorder=4)
        
        ax.set_xlim(-5, config.world_size + 5)
        ax.set_ylim(0.8, 1.2)
        ax.set_xlabel('Position')
        ax.set_title(f'Particle Filter Animation (Step {frame})')
        ax.legend(loc='upper left')
        ax.set_yticks([])
        ax.grid(True, alpha=0.3, axis='x')
    
    anim = FuncAnimation(fig, animate, frames=len(pf.particle_history),
                        interval=100, repeat=True)
    
    try:
        anim.save('particle_filter_animation.gif', writer='pillow', fps=10)
        print("✓ アニメーションを 'particle_filter_animation.gif' に保存しました")
    except Exception as e:
        print(f"アニメーション保存に失敗: {e}")
        print("  (PIL/Pillowがインストールされていない可能性があります)")
    
    plt.close(fig)


if __name__ == '__main__':
    print("=" * 60)
    print("モンテカルロ法による1次元ロボット自己位置推定シミュレーション")
    print("=" * 60)
    print()
    
    # シミュレーション実行
    robot, pf, config = simulate(num_steps=50)
    
    print("\n" + "=" * 60)
    print("可視化を生成中...")
    print("=" * 60)
    
    # 結果をプロット
    plot_results(robot, pf, config)
    
    # アニメーションを生成
    print("\nアニメーションを生成中...")
    plot_particle_animation(robot, pf, config)
