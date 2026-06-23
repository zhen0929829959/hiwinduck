## Git 提交與推送方式

本專案包含 Git submodule，如：

```text
src/Hiwin_libmodbus
src/realsense-ros
ubuntu22.04_ros2
```

因此修改一般檔案與修改 submodule 的提交方式不同。

---

### 一般專案檔案提交

在專案根目錄執行：

```bash
cd ~/work
git status
git add .
git commit -m "更新內容"
(git pull --rebase origin main)
git push origin main
```

---

### 修改 submodule 內容後提交

以 `src/Hiwin_libmodbus` 為例，需先提交 submodule 本身，再提交外層專案。

#### 1. 進入 submodule

```bash
cd ~/work/src/Hiwin_libmodbus
git status
```

#### 2. 提交 submodule 內的修改

```bash
git add .
git commit -m "修改 Hiwin_libmodbus"
git pull --rebase origin main
git push origin main
```

如果原始倉庫不是自己的帳號，可能沒有推送權限，需要先 Fork 倉庫，再修改遠端網址：

```bash
git remote set-url origin https://github.com/你的帳號/Hiwin_libmodbus.git
git push -u origin main
```

#### 3. 回到外層專案

```bash
cd ~/work
git status
```

此時通常會看到：

```text
modified: src/Hiwin_libmodbus (new commits)
```

#### 4. 提交 submodule 版本更新

```bash
git add src/Hiwin_libmodbus
git commit -m "Update Hiwin_libmodbus submodule"
git pull --rebase origin main
git push origin main
```

---

### 新電腦下載專案

第一次下載時使用：

```bash
git clone --recurse-submodules https://github.com/你的帳號/hiwinduck.git
```

如果已經 clone 過，但 submodule 尚未下載：

```bash
git submodule update --init --recursive
```

---

### 更新 submodule

```bash
git submodule update --remote --merge
```

更新完成後，外層專案仍需提交新的 submodule 版本：

```bash
git add .
git commit -m "Update submodules"
git push origin main
```

---

### 常見問題

#### 顯示 `modified content`

```text
modified: src/Hiwin_libmodbus (modified content)
```

代表 submodule 裡有尚未提交的修改。需先進入該 submodule 提交：

```bash
cd src/Hiwin_libmodbus
git add .
git commit -m "修改內容"
git push
```

#### 推送被拒絕 `fetch first`

```text
Updates were rejected because the remote contains work that you do not have locally
```

先同步遠端：

```bash
git pull --rebase origin main
git push origin main
```

#### GitHub 不接受超過 100 MB 的檔案

大型安裝檔不應上傳，例如：

```text
*.AppImage
```

可加入 `.gitignore`：

```gitignore
build/
install/
log/
*.AppImage
__pycache__/
*.pyc
```

如果大型檔案已經被加入提交紀錄，需要先移除：

```bash
git rm --cached 檔案名稱
git commit --amend --no-edit
git push origin main
```
