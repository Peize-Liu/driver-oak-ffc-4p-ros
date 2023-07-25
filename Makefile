all: oak_ffc

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build              - build all images"
	# @echo "   1. make pull             - pull all images"
	@echo "   1. make clean              - remove all images"
	@echo ""

oak_ffc:
	@docker build -t oak_ffc_img:latest -f ./oak_ffc_4p_ros.dockerfile ..

clean:
	@docker rmi -f oak_ffc_img:latest
