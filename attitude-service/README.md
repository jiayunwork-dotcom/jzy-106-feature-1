# 刚体姿态四元数积分服务 (attitude-quaternion-integrator)

一个常驻 HTTP 服务，只做一件事：以**四元数为主积分变量**，沿给定的机体角速度
时间序列逐步推进刚体姿态，输出末端姿态四元数、对应的欧拉角，以及全程姿态
范数偏离单位四元数的程度。不做位置/速度等惯导滤波，不做任何页面。

## 锁定的约定（响应中同样会原样写明）

| 项目 | 约定 |
|------|------|
| 四元数分量顺序 | `[w, x, y, z]` 标量在前（scalar-first） |
| 乘法 | Hamilton 约定：`ij=k, jk=i, ki=j` |
| 坐标系映射 | `q` 把矢量从机体系转到参考系：`v_n = q ⊗ v_b ⊗ q*` |
| 运动学方程 | `q̇ = 1/2 · q ⊗ (0, ω_b)`，**机体系角速度右乘**（不能左右弄反） |
| 积分器 | 经典四阶龙格–库塔（RK4），角速度在相邻采样间线性插值 |
| 归一化 | 每完成一步立即把四元数重新归一化，模长恒为 1 |
| 欧拉角顺序 | 固定为 **ZYX 内旋 = 偏航(yaw)/俯仰(pitch)/滚转(roll)**，弧度，范围 (-π, π] |
| 奇异处理 | 接近 \|pitch\|=90° 显式置 `singular`/`near_singular` 并给 `note`，绝不吐 NaN |

## 目录结构（按职责分文件，四元数代数与积分推进严格分开）

```
cmd/server/main.go                  进程入口：装配依赖、启动 HTTP
internal/quaternion/quaternion.go   四元数代数：乘法、共轭、归一化、单位判定
internal/integrator/integrator.go   RK4 推进、逐步归一化、单步范数漂移阈值
internal/euler/euler.go             四元数→ZYX 欧拉角 + 奇异检测
internal/store/store.go             命名角速度序列的内存存取（带拷贝隔离）
internal/validation/validation.go   请求参数合法性校验（带中文原因的错误码）
internal/api/                       轻薄接口层：解析请求、驱动积分、组织返回
```

## 一条命令构建并启动

```bash
docker compose up --build
# 服务监听 http://0.0.0.0:8080
# 若构建机访问 proxy.golang.org 不畅：
#   docker compose build --build-arg GOPROXY=https://goproxy.cn,direct
```

本地直接运行（Go 1.22）：

```bash
go run ./cmd/server            # 默认 :8080，可用 ATTITUDE_HTTP_ADDR 覆盖
```

健康检查：`GET /health`

## 接口

### `POST /api/v1/attitude/integrate`

请求体三种等价写法：

1. 逐采样对象：`samples: [{"t": 秒, "w": [wx, wy, wz]}]`
2. 并行数组：`timestamps: [...]` + `angular_velocities: [[...]]`（两者个数必须一致）
3. 引用已保存的命名序列：`series: "bench-yaw"`

| 字段 | 说明 |
|------|------|
| `q0` | 初始姿态四元数 `[w,x,y,z]`，必须是单位四元数（\|\|q\|-1\| ≤ 1e-6） |
| `samples` / `timestamps`+`angular_velocities` / `series` | 三选一 |
| `max_step_norm_drift` | 单步 `||q|-1|` 漂移阈值，默认 `1e-4` |
| `strict_drift` | `true`：超阈直接拒绝（HTTP 422）；`false`（默认）：告警但归一化后继续 |

`w` 的单位是 rad/s，且角速度在**机体系**下表达；时间戳单位秒，必须严格递增，
每个采样自带自己的时间步长（相邻时间戳之差）。

示例：纯绕竖直轴 0.5 rad/s 匀速旋转 10 s。

```bash
curl -s -X POST localhost:8080/api/v1/attitude/integrate \
  -H 'Content-Type: application/json' -d '{
    "q0": [1,0,0,0],
    "samples": [
      {"t":0,  "w":[0,0,0.5]},
      {"t":5,  "w":[0,0,0.5]},
      {"t":10, "w":[0,0,0.5]}
    ]
  }'
```

返回（节选）：

```json
{
  "quaternion_convention": { "...": "响应里完整写明上述约定 ..." },
  "initial_quaternion": [1, 0, 0, 0],
  "final_quaternion":   [-0.80114, 0, 0, 0.59847],
  "euler_angles": {
    "roll_rad": 0, "pitch_rad": 0, "yaw_rad": -1.2831853,
    "roll_deg": 0, "pitch_deg": 0, "yaw_deg": -73.521,
    "singular": false, "near_singular": false
  },
  "sample_count": 3, "step_count": 2, "elapsed_time_s": 10,
  "max_norm_drift": 2.7e-9, "norm_drift_threshold": 0.0001,
  "normalized_after_each_step": true,
  "warnings": []
}
```

> 末端 yaw=5 rad 超过 π，按 (-π, π] 折回为 5−2π ≈ −1.2832 rad。

### 命名序列（内存保存，重启即失效）

```bash
curl -X PUT   localhost:8080/api/v1/series/bench-yaw -H 'Content-Type: application/json' \
  -d '{"samples":[{"t":0.0,"w":[0,0,0.8]},{"t":0.5,"w":[0,0,0.8]}]}'
curl         localhost:8080/api/v1/series                 # 列出
curl         localhost:8080/api/v1/series/bench-yaw       # 读取
curl -X DELETE localhost:8080/api/v1/series/bench-yaw     # 删除
```

存取均做深拷贝：并发积分引用同一命名序列时互不影响，调用方也无法改写已存数据。

## 非法输入（积分开始前拦截，HTTP 400，带 `cause` 与中文 `detail`）

| cause | 触发条件 |
|-------|----------|
| `empty_sequence` | 序列为空 |
| `non_positive_step` | 某个时间步长 ≤ 0（时间戳必须严格递增） |
| `invalid_quaternion` | 初始四元数非单位，或含 NaN/Inf |
| `length_mismatch` | 角速度采样个数与时间戳个数不一致 |
| `invalid_timestamp` / `invalid_rate` | 时间戳或角速度分量为 NaN/Inf |
| `invalid_threshold` | 漂移阈值 ≤ 0 或非有限数 |
| `invalid_name` | 命名序列名称非法 |
| `not_found` | 引用的命名序列不存在 |

步长过大导致单步漂移超阈：默认进入 `warnings`；`strict_drift=true` 时 HTTP 422
拒绝（`cause=norm_drift_exceeded`），绝不闷头把姿态积失真。

## 自动化测试锁住的运动学事实

```bash
go test ./...            # 全量
go test -race ./...      # 含数据竞争检查
```

- `TestDoubleRateHalfStepSameAttitude`：整段角速度 ×2、每个时间步长 ÷2，末端姿态不变（总转角不变）。
- `TestNegatedRateIsReversedRotation`：整段角速度反号，末端姿态是原过程的逆转（`final(-ω)=conj(final(ω))`）；正程接逆程回到初始姿态。
- `TestFullTurnIsSameAttitude`：绕固定轴整转一圈，四元数回到初始朝向（允许整体差一个符号）。
- `TestZeroRateFreezesAttitude`：角速度恒零，姿态严格冻结，每步模长仍为 1。
- `TestConstantAxisRotationAngle`：恒定轴角速度，转角 = \|ω\|·t，转轴保持不变。
- `TestPureYawBenchmark`：纯绕竖直轴匀速旋转的手算基准，yaw 随时间线性增长、pitch/roll≈0，末端 yaw 钉死回归。
- 另有四元数代数、欧拉角往返与万向锁标志、参数校验、严格/非严格漂移策略、
  命名序列存取拷贝隔离、HTTP 全链路及 100 路并发积分互不串状态等测试。
