## Git 提交與推送方式

本專案包含 Git submodule，如：

```text
src/Hiwin_libmodbus
src/realsense-ros
ubuntu22.04_ros2
```

因此修改一般檔案與修改 submodule 的提交方式不同。

---

### 常用指令重點整理
開始工作前
```bash
cd ~/work
git pull --rebase origin main
git submodule update --init --recursive
```

```bash
cd src/Hiwin_libmodbus
git switch main
git pull --rebase origin main
```

修改完成後
```bash
cd ~/work/src/Hiwin_libmodbus
git add .
git commit -m "修改內容"
git pull --rebase origin main
git push origin main
```

```bash
cd ~/work
git add src/Hiwin_libmodbus
git commit -m "Update Hiwin_libmodbus submodule"
git pull --rebase origin main
git push origin main
```
組員下載更新
```bash
cd ~/work
git pull --rebase origin main
git submodule update --init --recursive
```
---
### 注記：
日常更新/統一版本 hiwinduck指向的submodule版本
```bash
git pull --rebase origin main
git submodule update --init --recursive
```
抓 Hiwin_libmodbus 最新版：
```bash
cd src/Hiwin_libmodbus
git pull --rebase origin main
```

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

### 下載專案

第一次下載時使用：

```bash
git clone --recurse-submodules https://github.com/zhen0929829959/hiwinduck.git
```


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

