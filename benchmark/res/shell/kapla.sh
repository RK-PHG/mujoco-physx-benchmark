#!/usr/bin/env bash

########################################################################################################################
##                                                                                                                    ##
## KAPLA TEST                                                                                                         ##
##                                                                                                                    ##
########################################################################################################################

echo "    __ __ ___    ____  __    ___       _______________________"
echo "   / //_//   |  / __ \/ /   /   |     /_  __/ ____/ ___/_  __/"
echo "  / ,<  / /| | / /_/ / /   / /| |      / / / __/  \__ \ / /   "
echo " / /| |/ ___ |/ ____/ /___/ ___ |     / / / /___ ___/ // /    "
echo "/_/ |_/_/  |_/_/   /_____/_/  |_|    /_/ /_____//____//_/     "


########################################################################################################################
# select sims
########################################################################################################################
source selectsim.sh


########################################################################################################################
# test
########################################################################################################################
#dt_array=( "0.0001" "0.0004""0.001" "0.004" "0.01" "0.04" "0.1" )
iter_array=( "10" "40" "100" "400" "1000" )
tol_array=( "0.1" "0.4" "0.01" "1e-4" "1e-6" "1e-8" "1e-10" )  # for mujoco
csv_file=$( date +"%Y-%m-%d-%H:%M:%S.csv" )

echo ""
echo "====================================================================="
# rai sim
if [ "$test_rai" == 'ON' ]; then
	if [ "$RAISIM_ON" == "ON" ]; then
		for erpon in true false
		do
			for num_iter in ${iter_array[@]}
			do
				timeout 6000 ../sim/raiSim/benchmark/RaiKaplaBenchmark --nogui --erp-on=$erpon --numiter=$num_iter --collapse --csv=$csv_file
			done
		done
	else
		echo "raisim is not built. turn on BENCHMARK_RAISIM option in cmake"
	fi
fi

# bullet sim
if [ "$test_bt" == 'ON' ]; then
	if [ "$BTSIM_ON" == "ON" ]; then
		for erpon in true false
		do
			for num_iter in ${iter_array[@]}
			do
				timeout 6000 ./bulletSim/bulletKaplaBenchmark --nogui --erp-on=$erpon --numiter=$num_iter --collapse --csv=$csv_file
			done
		done
	else
		echo "bulletsim is not built. turn on BENCHMARK_BULLETSIM option in cmake"
	fi
fi

# ode sim
if [ "$test_ode" == 'ON' ]; then
	if [ "$ODESIM_ON" == "ON" ] ; then
		for solver in quick
		do
			for erpon in true false
			do
				for num_iter in ${iter_array[@]}
				do
					timeout 6000 ../sim/odeSim/benchmark/OdeKaplaBenchmark --nogui --erp-on=$erpon --numiter=$num_iter --solver=$solver --collapse --csv=$csv_file
				done
			done
		done
	else
		echo "odesim is not built. turn on BENCHMARK_ODESIM option in cmake"
	fi
fi

# mujoco sim
if [ "$test_mjc" == 'ON' ]; then
	if [ "$MJCSIM_ON" == "ON" ] ; then
		for solver in pgs cg newton
		do
			for tolerance in ${tol_array[@]}
			do
				# note mujoco has no erp
				timeout 6000 ./mujocoSim/mujocoKaplaBenchmark --nogui --tolerance=$tolerance --solver=$solver --collapse --csv=$csv_file
			done
		done
	else
		echo "mujocosim is not built. turn on BENCHMARK_MUJOCOSIM option in cmake"
	fi
fi

# dart sim
if [ "$test_dart" == 'ON' ]; then
	if [ "$DARTSIM_ON" == "ON" ] ; then
		for solver in dantzig pgs
		do
			# note dart has no erp
			timeout 6000 ../sim/dartSim/benchmark/DartKaplaBenchmark --nogui --solver=$solver --collapse --csv=$csv_file
		done
	else
		echo "dartsim is not built. turn on BENCHMARK_DARTSIM option in cmake"
	fi
fi
