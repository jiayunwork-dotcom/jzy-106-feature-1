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
cmd/server/main.go                    进程入口：装配依赖、启动 HTTP
internal/quaternion/quaternion.go     四元数代数：乘法、共轭、归一化、单位判定
internal/integrator/integrator.go     RK4 推进、逐步归一化、单步范数漂移阈值；
                                      Simulate 与有状态轨迹共用同一个单步内核 Step
internal/euler/euler.go               四元数→ZYX 欧拉角 + 奇异检测
internal/store/store.go               命名角速度序列的内存存取（带拷贝隔离）
internal/trajectory/trajectory.go     有状态轨迹：按包追加、时间合并、存档回溯、
                                      补传窗口、重传幂等、版本号、轨迹注册表
internal/validation/validation.go     请求参数合法性校验（带中文原因的错误码）
internal/api/                         轻薄接口层：解析请求、组织返回
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

## 有状态轨迹（按包追加，晚到自动认回）

地面站一包一包下采样，没必要每包都从头重积。开一条**轨迹**之后，可以按包
追加采样：服务接着上一次的末端姿态往下推；晚到的补传包按时间戳插回正确
位置，只从插入点之前最近的一个存档往后重算，绝不从头积分。轨迹只放在
进程内存中，服务重启即失效。

运动学和校验与一次性积分完全同一份代码：轨迹的每一步都走
`integrator.Step`（`Simulate` 内部也调用它），历史时刻插值用的也是
`integrator.InterpolatedRate`，所以切包边界落在任何位置、包多乱序，最终
姿态都与合并后整段一次性提交**逐位相同**。

### 1. 开轨迹 `POST /api/v1/trajectories`

```bash
curl -s -X POST localhost:8080/api/v1/trajectories \
  -H 'Content-Type: application/json' -d '{
    "q0": [1,0,0,0],
    "max_step_norm_drift": 1e-4,
    "strict_drift": false
  }'
# -> {"trajectory_id":"a91dda66...","version":0, "max_late_samples":200, ...}
```

`q0`、`max_step_norm_drift`、`strict_drift` 的含义、取值范围与一次性积分
接口完全一致（非法初态/阈值用同样的 400 错误码拒绝）。

### 2. 按包追加 `POST /api/v1/trajectories/:id/append`

```bash
curl -s -X POST localhost:8080/api/v1/trajectories/$TID/append \
  -H 'Content-Type: application/json' -d '{
    "packet_seq": 1,
    "expected_version": 0,
    "samples": [
      {"t":0.0,"w":[0,0,0.5]},
      {"t":0.1,"w":[0,0,0.5]}
    ]
  }'
```

- `packet_seq`（必填）：包序号。采样同样支持 `samples` 或
  `timestamps`+`angular_velocities` 两种写法。
- `expected_version`（可选）：调用方看到的轨迹版本号，做乐观并发控制；
  对不上返回 409 `version_conflict`，整包不施加。
- 返回内容与一次性积分同字段同义：`final_quaternion`、`euler_angles`
  （含 `singular`/`near_singular`/`note`）、`max_norm_drift`、
  `norm_drift_threshold`、`normalized_after_each_step`、`warnings`
  （步号按**整条轨迹的全局步号**计），另加：
  - `version`：每成功并入一个含新采样的包 +1（重传不增）；
  - `duplicate_packet`：是否为幂等重传；
  - `inserted_samples`：本包新并入的采样数；
  - `recomputed_steps`：本次实际（重）积分的步数。

**切包无关**：同一段采样任意切包按序追加（最小一包一个采样），末端四元数
与整段提交逐位相同，跨包那一步的角速度插值与整段提交时相邻采样间的插值
是同一函数。

**晚到补传**：包内时间戳早于轨迹末尾时按时间插回，从插入点前最近存档往后
重算，`recomputed_steps` 报实际重算步数（小于总步数），结果与合并序列
整段提交逐位相同。补传只接受插入点之后现存采样不超过
`max_late_samples`（=200）的包；更早的旧包返回 422
`late_packet_out_of_window`，轨迹状态不动。

**重传幂等 / 冲突**：

| 情形 | 结果 |
|------|------|
| 同 `packet_seq`、内容相同 | `duplicate_packet=true`，不重复施加，版本/姿态不变 |
| 整包采样都已存在且值一致（序号不同） | 同样按幂等重传处理 |
| 同 `packet_seq`、内容不同 | 409 `packet_conflict`，整包拒绝 |
| 时间戳已存在但角速度不一致 | 409 `timestamp_conflict`，detail 报出冲突时间戳 |

**整包原子**：包内任一采样非法（时间步非正、NaN/Inf、包内时间戳重复/倒序
等）返回 400；严格模式下本包（含重算尾部）任一步漂移超阈返回 422
`norm_drift_exceeded`。两种情况下轨迹都回到收包前：末端姿态、步数、最大
漂移、存档、版本不留半截痕迹。

### 3. 历史时刻查询 `GET /api/v1/trajectories/:id/attitude?t=<秒>`

- `t` 恰好是采样时刻：返回该时刻存档姿态，`exact_sample=true`；
- 落在两采样之间：从前一个采样用同样的线性插值 + RK4 推进到 `t`
  （`exact_sample=false`，`bracket_step` 给出所在全局步号），结果等于把
  序列截到 `t`、在 `t` 处补一个按插值得出的采样后整段一次性提交的末端；
- 超出 `[start_time_s, end_time_s]` 或不是数字：400
  `time_out_of_range` / `invalid_timestamp`，并在 detail 给出当前覆盖范围。

### 4. 查看 / 列表 / 关闭删除

```bash
curl   localhost:8080/api/v1/trajectories            # 列出全部 id
curl   localhost:8080/api/v1/trajectories/$TID       # 查看状态（版本/步数/覆盖时间/最大漂移/已收包序号）
curl -X DELETE localhost:8080/api/v1/trajectories/$TID   # 关闭并删除
```

对已删除（或重启后丢失）的 id 再追加/查询，返回 404 `trajectory_not_found`。

**并发与隔离**：同一条轨迹的追加在轨迹内部串行化，并可再用
`expected_version` 做乐观锁，不会出现两包从同一旧末端各自下推；不同轨迹
之间状态、存档、告警、包序号互不影响。

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
- 有状态轨迹（`internal/trajectory` 与 HTTP 全链路）额外钉住：
  - 200 组随机切包（含单采样包）与整段提交末端四元数、最大漂移逐位相同；
  - 乱序补传合并后与整段提交逐位相同，且 `recomputed_steps` 小于总步数；
    补传窗口 ±200 个采样的边界与超窗拒绝；
  - 晚到插入后非严格告警的步号按全局步号重排，与一次性积分逐条一致；
  - 同序号重传幂等、同序号异内容/同时刻异速率冲突被拒且状态不变；
  - 严格模式下超阈包（含超阈发生在重算尾部）整体回滚；
  - 中间时刻查询等于截断并补插值采样后整段提交，越界带原因拒绝；
  - 同一轨迹并发追加不丢包、不重复施加，`expected_version` 乐观锁生效，
    不同轨迹彼此隔离（全部在 `-race` 下通过）。
