"""
1次元モンテカルロ自己位置推定シミュレータ (Particle Filter)

このモジュールは、粒子フィルタを用いて1次元空間でのロボット自己位置推定を実装しています。
センサー観測と移動信号からロボットの位置を推定します。
"""

import numpy as np
from dataclasses import dataclass


@dataclass
class Config:
    """シミュレーション設定"""
    world_size: float = 100.0  # 世界のサイズ [0, 100]
    num_particles: int = 1000  # 粒子の数
    process_noise: float = 1.0  # 移動モデルのノイズ標準偏差
    measurement_noise: float = 5.0  # センサーのノイズ標準偏差
    resample_threshold: float = 0.5  # リサンプリングの閾値
    landmark_positions: list = None  # ランドマークの位置
    
    def __post_init__(self):
        if self.landmark_positions is None:
            self.landmark_positions = [20.0, 40.0, 60.0, 80.0]


class ParticleFilter:
    """
    粒子フィルタを使用した1次元ロボット自己位置推定
    """
    
    def __init__(self, config: Config):
        """
        粒子フィルタの初期化
        
        Args:
            config: シミュレーション設定
        """
        self.config = config
        
        # 粒子の初期化（均等分布）
        self.particles = np.random.uniform(
            0, config.world_size, config.num_particles
        )
        self.weights = np.ones(config.num_particles) / config.num_particles
        self.particle_history = [self.particles.copy()]
        self.weight_history = [self.weights.copy()]
        
    def predict(self, control: float):
        """
        予測ステップ: 制御入力に基づいて粒子を更新
        
        Args:
            control: 制御入力（移動距離）
        """
        # ノイズを加えた移動
        noise = np.random.normal(0, self.config.process_noise, 
                                self.config.num_particles)
        self.particles = self.particles + control + noise
        
        # 境界処理（トーラス空間：ラップアラウンド）
        self.particles = self.particles % self.config.world_size
        
    def update(self, measurement: float):
        """
        更新ステップ: センサー観測に基づいて重みを更新
        
        Args:
            measurement: センサー観測値（最も近いランドマークまでの距離）
        """
        # 各粒子の位置から見た観測の尤度を計算
        likelihoods = np.zeros(self.config.num_particles)
        
        for i, particle_pos in enumerate(self.particles):
            # 粒子の位置から各ランドマークまでの最小距離を計算
            distances_to_landmarks = [
                abs(particle_pos - lm) for lm in self.config.landmark_positions
            ]
            min_distance = min(distances_to_landmarks)
            
            # ガウス尤度関数
            likelihood = self._gaussian_likelihood(
                measurement, min_distance, self.config.measurement_noise
            )
            likelihoods[i] = likelihood
        
        # 重みの更新
        self.weights = likelihoods * self.weights
        
        # 重みの正規化（ゼロ除算対策）
        weight_sum = np.sum(self.weights)
        if weight_sum > 0:
            self.weights = self.weights / weight_sum
        else:
            # 全て尤度がゼロの場合は均等重みに
            self.weights = np.ones(self.config.num_particles) / self.config.num_particles
    
    def resample(self):
        """
        リサンプリング: 有効粒子数に基づいて粒子を再抽出
        有効粒子数が少ない場合のみ実行
        """
        # 有効粒子数を計算
        effective_particle_count = 1.0 / np.sum(self.weights ** 2)
        
        # リサンプリングの判定
        if effective_particle_count < self.config.resample_threshold * self.config.num_particles:
            # 重みに基づいて粒子を再抽出
            indices = np.random.choice(
                self.config.num_particles,
                size=self.config.num_particles,
                p=self.weights,
                replace=True
            )
            self.particles = self.particles[indices]
            self.weights = np.ones(self.config.num_particles) / self.config.num_particles
    
    def filter_step(self, control: float, measurement: float):
        """
        1ステップのフィルタ処理を実行
        
        Args:
            control: 制御入力（移動距離）
            measurement: センサー観測値
        """
        self.predict(control)
        self.update(measurement)
        self.resample()
        
        # 履歴を保存
        self.particle_history.append(self.particles.copy())
        self.weight_history.append(self.weights.copy())
    
    def estimate_position(self) -> float:
        """
        粒子の重み付き平均から位置を推定
        
        Returns:
            推定位置
        """
        return np.average(self.particles, weights=self.weights)
    
    def get_confidence(self) -> float:
        """
        推定の信頼度を計算（粒子の分布の集中度）
        
        Returns:
            信頼度 [0, 1] （1に近いほど集中している）
        """
        return 1.0 / np.sqrt(np.sum(self.weights ** 2))
    
    @staticmethod
    def _gaussian_likelihood(x: float, mu: float, sigma: float) -> float:
        """
        ガウス尤度関数
        
        Args:
            x: 観測値
            mu: 平均
            sigma: 標準偏差
            
        Returns:
            尤度
        """
        exponent = -0.5 * ((x - mu) ** 2) / (sigma ** 2)
        return np.exp(exponent)


class Robot:
    """1次元ロボットのシミュレータ"""
    
    def __init__(self, true_position: float, config: Config):
        """
        ロボットの初期化
        
        Args:
            true_position: ロボットの真の位置
            config: シミュレーション設定
        """
        self.true_position = true_position
        self.config = config
        self.position_history = [true_position]
        self.measurement_history = []
    
    def move(self, control: float):
        """
        ロボットを移動させる
        
        Args:
            control: 制御入力（移動距離）
        """
        noise = np.random.normal(0, self.config.process_noise)
        self.true_position = (self.true_position + control + noise) % self.config.world_size
        self.position_history.append(self.true_position)
    
    def observe_landmark(self) -> float:
        """
        最も近いランドマークまでの距離を観測（ノイズ付き）
        
        Returns:
            観測値（距離）
        """
        distances = [abs(self.true_position - lm) for lm in self.config.landmark_positions]
        min_distance = min(distances)
        
        # ノイズを加える
        observation = min_distance + np.random.normal(0, self.config.measurement_noise)
        observation = max(0, observation)  # 負の距離は不可
        
        self.measurement_history.append(observation)
        return observation
