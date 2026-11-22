# 智能农装

小车控制、视觉与机构联调的团队协作仓库。

当前阶段：已建立协作入口，尚未导入小车控制程序。导入源码时保留原工程结构，并补充开发工具、依赖版本、编译和烧录方法。

## 新成员开始使用

1. 接受仓库协作邀请。
2. 克隆仓库：

```powershell
git clone https://github.com/sunnydman/zhineng-nongzhuang.git
Set-Location zhineng-nongzhuang
```

3. 阅读 [协作说明](CONTRIBUTING.md)，在独立分支上完成一个具体任务。

## 团队工作方式

- 用 [Issues](https://github.com/sunnydman/zhineng-nongzhuang/issues) 记录问题、负责人和验收条件。
- 用 `feat/简短英文描述`、`fix/简短英文描述` 分支开发和修复。
- 用 [Pull Requests](https://github.com/sunnydman/zhineng-nongzhuang/pulls) 提交改动，记录编译、仿真或实车验证结果。
- `main` 用于保存团队确认可用的版本；合并前检查改动和验证记录。

## 维护约定

源码、必要工程配置、依赖说明和调参配置一起管理。编译输出的忽略规则在确认实际开发工具后补充。

实车测试记录应能对应到提交编号，并写明硬件配置、关键参数、测试步骤和结果。尚未执行的验证明确标注为“未执行”。

仓库由 `sunnydman` 管理。新成员通过仓库 Settings → Collaborators 邀请；受邀成员接受后即可参与协作。
