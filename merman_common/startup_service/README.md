在有zqdfw.service 的文件目录下执行以下指令即可实现自启动

```bash
需要先根据机器人编号，在zqdfw.service中的第13行设置对应的domain_id
sudo chmod 777 zqdfw.service
sudo cp zqdfw.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable zqdfw.service
```
