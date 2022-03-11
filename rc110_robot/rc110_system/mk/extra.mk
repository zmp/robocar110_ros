start:
	systemctl --user restart rc110-roscore rc110

stop:
	systemctl --user stop rc110-roscore rc110

status:
	systemctl --user -n200 status rc110

status-core:
	systemctl --user -n200 status rc110-roscore

list-masters:
	@rosservice call /master_discovery/list_masters | grep -o -P '(?<=name: ").*(?=.local")'
