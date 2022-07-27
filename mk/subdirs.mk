#
# Adding make targets from subdirectories.
#

# usual targets: <>, deps-<>, run-<>, etc...
adv_node_dirs := $(subst /GNUmakefile,,$(wildcard rc110_*/rc110_*/GNUmakefile))  # all nodes that contain GNUmakefile
adv_nodes := $(subst _,-,$(subst rc110_,,$(sort $(notdir $(adv_node_dirs)))))    # get their names

$(adv_nodes):
	$(MAKE) -C $(wildcard rc110_*/$(subst -,_,$(addprefix rc110_,$@)))

deps_adv_nodes := $(addprefix deps-,$(adv_nodes))
$(deps_adv_nodes):
	$(MAKE) deps -C $(wildcard rc110_*/$(subst -,_,$(addprefix rc110_,$(subst deps-,,$@))))

run_adv_nodes := $(addprefix run-,$(adv_nodes))
$(run_adv_nodes):
	$(MAKE) run -C $(wildcard rc110_*/$(subst -,_,$(addprefix rc110_,$(subst run-,,$@))))

show_adv_nodes := $(addprefix show-,$(adv_nodes))
$(show_adv_nodes):
	$(MAKE) show -C $(wildcard rc110_*/$(subst -,_,$(addprefix rc110_,$(subst show-,,$@))))

# extra targets
include $(wildcard rc110_*/*/mk/extra.mk)  # all extra.mk files
