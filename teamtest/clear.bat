@echo off
echo 正在清理 Keil 编译产生的垃圾文件...

:: 删除编译输出文件夹（如果存在）
rd /s /q Objects
rd /s /q Listings

:: 删除根目录及子目录下常见的垃圾后缀文件
del *.o *.d *.crf *.dep *.lnp *.axf *.htm *.map *.lst *.bak *.sct /s
del *.iex *.sfr *.txt *.m51 *.m66 *.iou *.hex /s

:: 删除 Keil 自动生成的布局和调试配置文件（可选，这俩经常变动导致 git 冲突）
del *.uvguix.* /s
del *.uvoptx /s
del JLinkLog.txt /s

echo 清理完成！
pause