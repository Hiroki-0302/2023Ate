# 2023 長野高専ロボコン 制御フレームワーク

## このフレームワークの目的
通信や、モーター等の制御などを簡単に行えるようにすること。

## FLINTからの変更点
- 自分でも理解できないレベルに複雑化し、保守ができない・調整が間に合わない恐れがあったため一から再設計した。
- FreeRTOS部分をすべて排除し、マルチタスクを撤廃した。
- 通信・アクチュエーター等制御部分をライブラリとして分離した。->NRUnifiedリポジトリに移動
  - チーム間でコードの変更を適用しやすくするため

## 実装段階
[実装状況](実装状況.md)

## 利用方法
[利用方法](利用方法.md)

## ライセンス

