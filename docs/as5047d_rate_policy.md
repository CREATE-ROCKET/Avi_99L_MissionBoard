# AS5047D取得レート制約

## 1. 優先順位

本書は2026-08-16時点の99L AS5047D取得レート決定であり、既存の`docs/05_実装仕様.md`、`docs/characterization_campaign.md`、`docs/imu_control_heartbeat.md`等に残る5 kHz / 10 kHz取得記述と矛盾する場合は本書を優先する。

AS5047Dは最大更新レート2.5 kHzであるため、99Lでは仕様上限ぎりぎりを使わず、新規取得を次の2レートだけに限定する。

- 1000 Hz
- 2000 Hz

5000 Hz / 10000 Hzで新規AS5047D captureを行わない。

## 2. Characterization

characterizationのrate-checkおよびfull captureで実際にAS5047D samplerを開始できるのは1000 Hz / 2000 Hzだけとする。

`EncoderRate::Hz5000`はV5の旧ログ・旧campaignとの互換用enumとして当面残すが、`EncoderSampler::begin()`は5000 Hzを`ESP_ERR_INVALID_ARG`で拒否し、SPI初期化やpipelined captureを開始しない。

Stage完了条件は1000 Hz / 2000 Hzのrate-check解決と1000 Hz full完了だけを見る。旧5 kHz campaign slotはStage完了条件に含めない。

1000 Hzは必須レートとする。2000 Hzはrate-check結果に基づいてaccepted / unsupportedを決定する。2000 Hzがunsupportedでも1000 Hz fullが正常であればcampaignは継続可能とする。

## 3. Production

productionのAS5047D capture候補は2000 Hzのみとし、1 kHz estimator/controlへ渡す。5 kHz captureをproduction候補から除外する。

飛行中にdiagnostic目的でpipelined readを周期停止しない方針は変更しない。

## 4. 旧ログ互換

V5 wire layout、`EncoderRate::Hz5000`の数値、`kMaximumEncoderSamplesPerEpoch=5`は過去ログのdecode互換を壊さないため直ちには削除しない。

これは5 kHz acquisitionを現在サポートする意味ではない。新規実機取得の許可条件と旧wire valueの認識を分離して扱う。

## 5. Realtime capture timing

PRECHECK7.5では1 kHz raw capture自体はcompleteだった一方、consumerが前epoch処理を終える前に次alarmへ到達してrelease deadlineを落としていた。このためconsumerのqueue drainは「空になるまでproducerを追う」方式を禁止し、drain開始時にSPSC ringのproducer headをsnapshotして、その時点で公開済みだったsampleだけを有限個処理する。snapshot後にproducerがpublishしたsampleは次consumer cycleへ残す。future sampleを現在epochへ借りることはなく、epoch所属は引き続きactual capture timestampで決める。

V5 recordのfull validationは`char_runtime`では実施せず、immutable recordをwriter queueへ値copyした後、`char_writer`がwire encode直前に`validateRecordStrict()`を1回だけ実施する。motor safety、position guard、deadline、queue overflow、encoder transport/status errorは従来どおりrealtime側で直接監視し、writer validation failureも既存failure notificationで安全停止へ伝播させる。native/offline validationは従来どおりstrictである。

既存`CHAR_RATE_STAGE`の`record-validate-max-us`はrealtime pathからvalidationを外した後は0を基本とする。strict validationはwriter側へ移動したため、このfieldをwriter CPU時間として再解釈しない。

### 5.1. Trigger phase

1 kHzは従来どおりepoch内500 usでtriggerする。

2 kHzはPRECHECK7.5でscheduled→actual capture最大347 usが観測され、250/750 us triggerでは500 us slot境界を越えた。このため2 kHzのprovisional triggerを120/620 usへ前倒しする。120 usは100 us command deadlineより後に置き、encoder taskがcommand apply deadline内を直接奪わないようにする。

この120/620 usは採用済みflight定数ではなく実機qualification用のprovisional phaseである。actual capture timestampが各half-open slot内へ入り、repeated/skipped/deadlineが0になることをrate-checkで確認できない場合、2 kHzはunsupportedのままとする。deadlineやslot境界を緩めて成立扱いにしてはならない。

### 5.2. Task affinityとwriter throughput

PRECHECK7.8では`char_runtime`をCore 0へ分離したことで、1 kHzの`consumer-work-max-us`は644 us、`consumer-wait-late-max-us`は0まで改善し、2 kHzでもraw epochはcompleteになった。一方、1000/2000 Hzとも約233 recordでwriter queueがoverflowした。

characterizationではtask affinityを次のように固定する。

- Core 0: `char_runtime` priority 21、`char_vbus` priority 12、`char_console` priority 5
- Core 1: `char_encoder` priority 23、`char_writer` priority 10

`char_encoder`は同Coreのwriterより常に優先し、AS5047D captureをSD書込みより優先する。

PRECHECK7.9では1 kHzが10秒・10000 recordを`writer-queue-overflow=0`で完走し、queue high-waterは56だった。2 kHz raw captureもcompleteだったが、Core 1上で2 kHz encoder taskにpreemptされるwriterがqueue depth 128を使い切り、1615 record時点でoverflowした。SD/FATの瞬間stallをcharacterizationのRAM bufferで吸収することは許可するため、writer queue depthを512 recordsへ拡張する。

queue拡張後も`queue-high-water`を必ず記録し、10秒run中に512へ単調に近づく場合は平均writer throughput不足として不合格とする。queue拡張は持続速度不足を合格扱いにするためには使用しない。

SDMMC mountは`char_runtime`から行わず、Core 1へpinした`char_writer` task自身のstartupで実施する。これによりstorage driverの初期化・interrupt allocationをrealtime consumer側Core 0から分離する。mount成否はstartup semaphoreで`LogWriterV5::initialize()`へ返し、失敗時はcaptureを開始しない。

rate-check終了時は`CHAR_WRITER_TIMING`を出力する。

```text
queue-depth
queue-high-water
batch-capacity
max-batch-records
batch-count
preallocate-us
planned-bytes
validate-max-us
validate-total-us
validate-avg-us
encode-max-us
encode-total-us
encode-avg-us
fwrite-max-us
fwrite-total-us
fwrite-avg-us
records-written
```

`fwrite-max-us`はSD/FATの瞬間stall、`fwrite-avg-us`とqueue high-waterの推移は持続throughputの判定に使用する。1000 Hz consumer record生成はencoder raw rateが2 kHzでも1 record/msのままである。

### 5.3. PRECHECK8後のSD書込み契約

PRECHECK8ではqueue depth 512でも、1 kHzで`fwrite-max-us=951768`、2 kHzで`fwrite-max-us=532719`の長時間stallが観測された。1 kHzは9328 record、2 kHzは535 recordでqueue high-waterが512へ到達し、いずれもwriter queue overflowで停止した。raw encoder側は停止まで`repeated=0`、`skipped=0`、`steady-incomplete=0`であったため、queueをさらに拡大してstorage throughput不足を隠す方針は採らない。

writer queue depthは512 recordsのまま維持し、1回のwriter batch上限を16 recordsから64 recordsへ拡大する。queueにbacklogがある場合は最大20 KiB級の連続V5 recordを一度の`fwrite`へまとめ、FAT/VFS/SDへの小write回数を減らす。

各runではmotor、AS5047D sampler、consumer GPTimerを開始する前に、planned file sizeを

```text
V5 header + expected_epochs * V5 record + V5 footer
```

として`ftruncate()`で事前確保し、`fsync()`まで完了させる。ESP-IDF FatFs VFSのfile拡張は不足領域を実際にzero writeしてclusterを確保するため、motor run中のFAT cluster allocationをrun開始前へ移す。preallocation完了後に新しい`epoch_zero`を決め、characterizationでは1.5 sのstart leadを確保する。

runがabortしてplanned record数に達しなかった場合もzero tailをV5 logとして残さない。footerを書き終えた実file positionへ終了時に`ftruncate()`し、`fsync()`してからcloseする。したがってstrict decoderが見るfile layoutは従来どおり`header + actual records + footer`である。

V5 CRC32はpolynomial `0xEDB88320`、初期/final XOR、`previous_crc` chainingを変更せず、1 byteごとの8-bit loopから256-entry table lookupへ変更する。writerのfile CRCはrecord単位で関数を呼び直さず、同一batchの連結bytesを1回のchained CRCへ渡す。既存Python golden fixtureのCRC固定値とencode/decode byte一致を互換条件とし、V5 schema、record size、Spica importerを変更しない。

preallocation、batch拡張、CRC高速化によっても100 us command/release deadline、half-open epoch/slot境界、queue overflow時のfail-safeを緩和しない。次のrate-checkでwriter overflowを解消した後もdeadline missが残る場合に限り、GPTimer interrupt priority/affinityを別変更として評価する。

## 6. Characterization build最適化

characterization firmwareはtiming qualification専用buildとして、ESP-IDF framework componentには公式の`CONFIG_COMPILER_OPTIMIZATION_PERF=y`を使用し、`-O2`のまま維持する。framework全体へPlatformIO `build_flags`で`-O3`を注入してはならない。

MissionBoard自前の`src` componentだけは、characterization用CMake option `AVI_99L_CHARACTERIZATION_BUILD=ON`のとき`target_compile_options(... -O3)`を追加する。これにより`profile_runner`、`fixed_epoch_assembler`、`record_validation`、`log_writer_v5`、`encoder_sampler`等の自前処理を`-O3`で最適化しつつ、ESP-IDF UART/FreeRTOS/SDMMC等はEspressifが想定する`-O2`でbuildする。

ESP-IDF framework全体のcompile-time/link-time LTOは使用しない。LTOによるframework task stack使用量やdriverコード生成の変化をcharacterization timing qualificationへ混入させないためである。

`-Ofast`およびfast-math系optionも使用しない。`std::isfinite`、NaN/Inf、浮動小数点比較等の安全判定の意味論を維持するためである。

2026-08-16の全component `-O3`試行ではESP-IDF `esp_driver_uart/src/uart.c`がGCCの`-Warray-bounds`を発生させ、frameworkの`-Werror`でbuild停止した。このため以後はcomponent-local `-O3`を正式なcharacterization build方針とする。

最適化変更後のbinaryは別firmware SHAとして扱い、1/2 kHz rate-check、full 1 kHz、writer throughput、position guard、安全停止を再qualificationする。deadlineやacceptance基準は最適化のために緩和しない。
