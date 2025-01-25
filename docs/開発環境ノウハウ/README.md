
# 開発環境ノウハウ

## 個人情報の保護のための対策

* 問題: Wifiのパスワードなどの個人情報を含むファイルは、リポジトリに含めないようにする必要がありますが、**Key.h**ファイルに記述対応しています。

**Key.hファイルを管理対象から外すため、<font color="Red">下記のコマンドを実行してください。</font>**

<details open>
<summary>gitの管理対象から外すコマンド</summary>

```bash
git update-index --skip-worktree    ./Meridian_LITE_on_Arduino/src/keys.h
git update-index --skip-worktree    ./Meridian_LITE_for_ESP32/src/keys.h
```

</details>

<details>
<summary>gitの管理対象に戻したい場合のコマンド</summary>

```bash
git update-index --no-skip-worktree ./Meridian_LITE_on_Arduino/src/keys.h
git update-index --no-skip-worktree ./Meridian_LITE_for_ESP32/src/keys.h
```

</details>

