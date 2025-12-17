.PHONY: restart start stop log port

restart:
	systemctl restart mystartup.service

start:
	systemctl start mystartup.service

stop:
	systemctl stop mystartup.service

log:
	journalctl -u mystartup.service -f

clear_log:
	journalctl --vacuum-time=1min

port:
	netstat -tulpn | grep :8888