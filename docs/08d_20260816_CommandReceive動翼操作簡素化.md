# 2026-08-16 CommandReceive 動翼操作簡素化

Vault `CREATE/99L Mission Board/04d_CommandReceive動翼操作簡素化.md` に従う。

## production command

CommandReceiveで地上局へ公開する動翼の駆動操作は次の2つだけとする。

- `0x10 FinReleaseHold`: pending move/holdを解除してFreeへ移行
- `0x13 FinHoldCurrent`: args[0..5]=0のみ受理し、受信時の現在角をtargetとして保持

`0x13`はproduction runtimeの既存relative-move pathを0 deg移動として使用する。現在角がtargetになり、到達判定後は`PositionHold`へ移行する。任意相対角は受理しない。

旧`0x12 StartFinZeroHold`は`NotSupported`とする。

## Fin zero

`0x11 SetFinZero`は駆動操作ではなく飛行前設定として残す。現行runtimeでは`FinZeroConfigured=true`を成立させる経路であり、通常`StartSequence`のreadinessに必要となる。

したがって、地上局UI上では動翼の通常操作を「現在角保持」「保持解除」に限定し、`SetFinZero`は校正・設定操作として分離して表示する。

## safety

`FinHoldCurrent`は既存のfin availability、FinZeroConfigured、encoder/rate validity、motor availability、software limit、overtravel interlockをそのまま通す。条件不足を自動bypassしない。
