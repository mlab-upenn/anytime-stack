#!/usr/bin/env bash

# ASCII color sequences
# credits: https://stackoverflow.com/questions/5947742/how-to-change-the-output-color-of-echo-in-linux
# see also: https://unix.stackexchange.com/questions/269077/tput-setaf-color-table-how-to-determine-color-codes
# to get all 256 colors:
#   for c in {0..255}; do tput setaf $c; tput setaf $c | cat -v; echo =$c; done
red=$(tput setaf 1)
green=$(tput setaf 2)
cyan=$(tput setaf 6)
gray=$(tput setaf 8)
bold=$(tput bold)
reset=$(tput sgr0)

print_help() {
	name=./$(basename "$0")
	echo "Compile and run the given target."
	echo "Usage: $name [options] target [target run arguments]"
	echo "Note: If target is set to all or clean nothing is run (only build is invoked)."
	echo "Options:"
	echo "  -h, --help > print help and exit"
	echo "  -v, --valgrind > run with Valgrind"
	echo "  -r, --run-in-root > runs program with project root as working directory"
	# echo "  -t, --type > "
	echo "  note: options might be given in any order"
}

format_boolean() {
	if [[ $1 -eq 1 ]]; then
		echo "${2}YES${reset}"
	else
		echo "${3}NO${reset}"
	fi
}

type="debug"

if [[ $(uname) == "Darwin" ]]; then
	platform="macos"
else
	platform="linux"
fi

run_in_root=0
valgrind=0
target=""
run_arguments=()
build_options=()

# echo "parsing options: $*"
# parse CLI options: [options] target [target run arguments]
while [[ -n $1 && $1 =~ ^- ]]; do

	case $1 in
	-h | --help)
		print_help
		exit 0
		;;
	-v | --valgrind)
		valgrind=1
		;;
	-r | --run_in_root)
		run_in_root=1
		;;
	*)
		echo "${red}Unknown option ${1}${reset}"
		print_help
		exit 1
		;;
	esac

	# move to the next option
	shift

done
target="$1"
shift
run_arguments+=("$@")

if [[ -z $target ]]; then
	echo "${red}Target to compile and run must be specified${reset}"
	print_help
	exit 1
fi

# build_dir="cmake-build-${type}-${platform}"
build_dir="cmake-build-${type}"

# compile
build_options+=("--build")
build_options+=("$build_dir")
# short option -t is not available in CMake 3.13
# build_options+=("-t")
build_options+=("--target")
build_options+=("$target")
echo "${gray}Compiling ${cyan}cmake ${build_options[*]}${gray} ...${reset}"
if ! cmake "${build_options[@]}"; then
	echo "${red}Compilation failed${reset}"
	exit 1
fi

if [[ $target == "clean" || $target == "all" ]]; then
	echo "Nothing to run (considering clean and all as special targets)."
	exit 0
fi

# run
if [[ $run_in_root -eq 1 ]]; then
	working_directory="."
	program="${build_dir}/${target}"
else
	working_directory="$build_dir"
	program="./${target}"
fi
echo "${gray}Running with these options:${reset}"
echo "${gray}- valgrind: $(format_boolean "$valgrind" "$green" "$red")${reset}"
echo "${gray}- run_in_root: $(format_boolean "$run_in_root" "$green" "$red")${reset}"
echo "${gray}- arguments: ${cyan}${run_arguments[*]}${reset}"
cd "$working_directory" || exit 1
if [[ $valgrind -eq 1 ]]; then
	valgrind --leak-check=full --track-origins=yes "$program" "${run_arguments[@]}"
else
	$program "${run_arguments[@]}"
fi
