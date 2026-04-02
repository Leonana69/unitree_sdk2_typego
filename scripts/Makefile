.PHONY: restart start stop log port

restart:
	$(if $(CAST_IP), systemctl set-environment CAST_IP=$(CAST_IP),)
	systemctl restart mystartup.service

start:
	$(if $(CAST_IP), systemctl set-environment CAST_IP=$(CAST_IP),)
	systemctl start mystartup.service

stop:
	systemctl stop mystartup.service

log:
	journalctl -u mystartup.service -f

clear_log:
	journalctl --vacuum-time=1min

port:
	netstat -tulpn | grep :8888