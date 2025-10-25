## 仓库使用手册

---

### **一、邀请团队成员加入仓库 (Repo Owner 操作)**

由仓库拥有者操作，将团队成员邀请进来作为项目的协作者 (Collaborator)。

1. 在 GitHub 上打开你的仓库页面。
2. 点击仓库名称下的 **Settings** 选项卡。
3. 在左侧菜单中，点击 **Collaborators**。
4. 点击 **Add people** 按钮。
5. 输入团队成员的 GitHub 用户名、全名或邮箱地址，在下拉列表中选择正确的用户，然后点击 **Add [username] to this repository**。
6. 团队成员会收到一封邀请邮件，接受邀请后，就拥有了向这个仓库推送代码的权限。

### **二、团队成员克隆仓库到本地 (团队成员操作)**

团队成员需要将远程仓库复制一份到自己的电脑上。

1. 打开 GitHub 仓库主页，点击绿色的 **<> Code** 按钮。

2. 复制 HTTPS 或 SSH 链接。

3. 在自己的电脑上打开终端（或 Git Bash），执行以下命令：

   ```bash
   # 推荐使用 SSH 方式
   git clone git@github.com:你的用户名/你的仓库名.git
   
   # 或者使用 HTTPS 方式
   git clone https://github.com/你的用户名/你的仓库名.git
   ```

现在，所有成员的本地电脑上都有了这个项目的完整副本。

### **三、核心开发工作流 (所有人重复此流程)**

#### A. 开始新任务前：同步最新代码

在开始写任何新代码之前，务必确保你的本地 `main` (或 `master`) 分支是最新版本。

```bash
# 1. 切换到主分支
git checkout main

# 2. 从远程仓库拉取最新的代码并合并到本地
git pull origin main
```

#### B. 为新任务创建分支

为每一个新功能或 bug 修复创建一个新的分支。分支名称要有意义，比如 `feature/user-login` 或 `fix/button-style`。

```bash
# 从最新的 main 分支创建并切换到一个名为 feature/user-login 的新分支
git checkout -b feature/user-login
```

现在，团队成员就可以在这个 `feature/user-login` 分支上安心地编写代码了，他的所有操作都不会影响到 `main` 分支和其他人。

#### C. 在新分支上开发与提交

在 `feature/user-login` 分支上进行编码、修改、测试... 当完成一个阶段性的工作后，就进行一次提交 (commit)。

```bash
# 1. 查看当前文件状态，会列出你修改过的文件
git status

# 2. 将你想要提交的文件添加到暂存区 (staging area)
# 添加所有修改过的文件
git add .
# 或者只添加某个特定文件
git add src/components/LoginForm.vue

# 3. 提交暂存区的文件到本地仓库，并附上清晰的说明
git commit -m "feat: 完成用户登录页面的UI布局"
```

#### D. 推送分支到远程仓库

当功能开发完毕或需要协助时，需要将你的本地分支推送到 GitHub 远程仓库。

```bash
# 第一次推送这个新分支时，需要设置上游 (upstream)
git push --set-upstream origin feature/user-login

# 之后再有新的 commit 推送到这个分支时，就可以简化为：
git push
```

推送成功后，在 GitHub 的仓库页面上就能看到这个新的分支了。

### **四、创建合并请求 (Pull Request)**

分支推送到远程后，就可以发起一个 Pull Request (简称 PR)，请求别人审查你的代码，并最终把它合并到 `main` 分支。

1. 在 GitHub 仓库页面，会看到一个黄色的提示条，提示刚刚推送了一个新分支，旁边会有一个 **Compare & pull request** 按钮，点击它。
2. 或者，可以切换到你的分支，点击 **Contribute** -> **Open pull request**。
3. 在打开的页面中：
   - **标题 (Title)**：清晰地描述这个 PR 做了什么，比如 “新增用户登录功能”。
   - **描述 (Description)**：详细说明你做了哪些改动、为什么这么做、如何测试等。如果修复了某个 issue，可以在这里写 `Closes #123` 来关联。
   - **Reviewers**：在右侧选择希望审查你代码的同事。
4. 确认无误后，点击 **Create pull request**。

### **五、**代码审查与合并

1. **审查 (Review)**：被指定的同事会收到通知。他们会查看你提交的代码文件，可以在具体的代码行上发表评论、提出修改建议。
2. **讨论与修改 (Discuss & Update)**：你会收到审查的反馈。如果需要修改，就在你本地的 `feature/user-login` 分支上继续修改、commit、然后再次 `git push`。新的 commit 会自动追加到这个 PR 上。
3. **合并 (Merge)**：当所有人都觉得代码没有问题，给出了 "Approve" (批准) 后，项目负责人（或者你自己）就可以点击绿色的 **Merge pull request** 按钮，将你的分支代码正式合入 `main` 分支。
4. **删除分支 (Delete Branch)**：合并后，GitHub 会提示你是否要删除这个已经完成使命的分支。通常建议删除，保持仓库干净。这只会删除远程分支，你的本地分支还存在。

### **六、清理本地分支**

PR 被合并后，其他人就可以在自己的本地更新代码了，而自己也可以清理掉本地的临时分支。

```bash
# 1. 切换回主分支
git checkout main

# 2. 拉取远程最新的代码（包含了刚刚合并进去的功能）
git pull origin main

# 3. 删除已经没用的本地分支
git branch -d feature/user-login
```

