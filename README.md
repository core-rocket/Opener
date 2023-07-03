# Flight Detection and Parachute Deployment System for Hybrid Rockets

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

## 使用方法
* Openerクラスを継承したクラス(`myOpener`)の作成
    * シミュレーション結果に応じて`protected`のパラメータを設定する．
    * `open()`，`close()`の関数を実装する．ハードウェアで必要であれば`init()`も実装．
* myOpenerインスタンスの利用
    * `opener_100Hz()`を100Hzで呼び出してセンサーデータを渡すと，自動的に判定が行われ`open()`が呼び出される．
    * 離床から開放までの時間のシム値に関しては，風向風速や射角に依存し直前にアップリンクする必要があるため，`open_threshold_time_ms()`で値を上書きできる．
    * その他のコマンドについてはサンプルコードを参照．


[Openerクラスのドキュメント](https://core-rocket.github.io/Opener/classOPENER.html)

### サンプルコード
https://github.com/core-rocket/Opener/blob/03a7ea9a9297b0590195f735b6251cbbd7a642fb/examples/simple_Opener/simple_Opener.ino#L1-L55

[CORE CAN Protocol](https://github.com/core-rocket/CCP)に対応したサンプルコードは[examples/CCP_Opener](https://github.com/core-rocket/Opener/blob/main/examples/CCP_Opener/CCP_Opener.ino)にあります．

### パラメータの設定


## 設計思想

### 状態遷移の設計
### ソフトウェアの実装

<!--

public,private,protectedの説明
ゲッターとセッター
-->