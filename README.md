# Opener

## 状態遷移

| フェーズ定義 | 概要 | 該当フェーズへの移行条件 |
|:-----------:|:----:|:----------------------:|
| CHECK | 打上げ前の動作確認 | 起動時・移行コマンドの受信 |
| READY | 点火待機 | (キースイッチ・)移行コマンドの受信 |
| FLIGHT | 点火後 | 離床判定 |
| OPENED | 減速機構作動後 | 開放判定 |

### 離床判定
![](doc/lift_off.png)
### 開放判定
![](doc/open.png)

## 設計思想
### 状態遷移の設計

### ソフトウェアの実装

## 使用方法

[Openerクラスのドキュメント](https://core-rocket.github.io/Opener/classOPENER.html)

### サンプルコード
https://github.com/core-rocket/Opener/blob/03a7ea9a9297b0590195f735b6251cbbd7a642fb/examples/simple_Opener/simple_Opener.ino#L1-L55

### パラメータの設定
