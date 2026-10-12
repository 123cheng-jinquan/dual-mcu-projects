# 09 · Git 远程配置说明（网络受限环境）

> 这份是个**操作备忘**，不是技术文档。将来换机器或网络恢复时看它。

## 本机的实际情况

这台机器上 **`github.com` 的 DNS 解析会被打断**：

```
[System.Net.Dns]::GetHostAddresses('github.com')
  → No such host is known            ✗

github.com:22            → 解析失败，连不上   ✗
ssh.github.com:22        → 通                ✓
ssh.github.com:443       → 通                ✓
```

原因：GitHub 的主域名在国内网络环境下常被 DNS 污染/阻断，
但它的**备用 SSH 入口 `ssh.github.com:443`** 通常可用
（这个入口本来就是 GitHub 为穿防火墙场景提供的）。

## 因此本仓库的 remote 被改成了备用入口

```bash
git remote -v
# origin  ssh://git@ssh.github.com:443/123cheng-jinquan/modbus-vibration-gateway.git
```

**注意**：这是 `ssh://` 形式的 URL（带端口 443），
**不是**普通的 `git@github.com:...` 形式。

## 还有一个本机设置

```bash
git config core.sshCommand
# ssh -o UserKnownHostsFile=D:/MyProject/.git_hostkeys -o StrictHostKeyChecking=accept-new
```

**为什么需要它**：写沙箱环境时无法写 `~/.ssh/known_hosts`，
所以指定了一个**可写的主机密钥文件** `.git_hostkeys`（已加入 `.gitignore`）。

> ⚠️ **在正常情况下（比如你自己的机器、网络正常时）这两项都不需要**。
> 如果你换到网络正常的机器，建议改回标准形式：
>
> ```bash
> git remote set-url origin git@github.com:123cheng-jinquan/modbus-vibration-gateway.git
> git config --unset core.sshCommand
> ```

## 以后推送

```bash
cd D:\MyProject
git add -A
git commit -m "说明"
git push
```

如果哪天 `push` 又报 `Could not resolve hostname github.com`，
**说明备用入口也失效了**，可以按顺序试：

| 试什么 | 命令 |
|---|---|
| ① 确认备用入口还通 | `ssh -T -p 443 git@ssh.github.com` |
| ② 换 Gitee 做镜像 | `git remote add gitee <你的 gitee 地址>`，然后 `git push gitee main` |
| ③ 等网络恢复 | 本地提交是安全的，随时能推 |