all: schunk_sdh

schunk_sdh:
	# create link to lib depending on version (32-bit/64-bit)
	cd common/lib && ln -sf `uname -m`/* .

include $(shell rospack find mk)/cmake.mk
