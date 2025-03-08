#!/usr/bin/env bash

########################################################################################################################
##                                                                                                                    ##
## ELASTIC 666 BENCHMARK                                                                                                      ##
##                                                                                                                    ##
########################################################################################################################

echo "    ________    ___   _____________________   _____ _____ _____"
echo "   / ____/ /   /   | / ___/_  __/  _/ ____/  / ___// ___// ___/"
echo "  / __/ / /   / /| | \__ \ / /  / // /      / __ \/ __ \/ __ \ "
echo " / /___/ /___/ ___ |___/ // / _/ // /___   / /_/ / /_/ / /_/ / "
echo "/_____/_____/_/  |_/____//_/ /___/\____/   \____/\____/\____/  "


########################################################################################################################
# select sims
########################################################################################################################
source selectsim.sh


########################################################################################################################
# test
########################################################################################################################
dt_array=( "0.0001" "0.0004" "0.001" "0.004" "0.01" "0.04" "0.1" )
csv_file=$( date +"%Y-%m-%d-%H:%M:%S.csv" )

echo ""
echo "====================================================================="
# benchmark test
for dt in ${dt_array[@]}
do
    # rai sim
    if [ "$test_rai" == 'ON' ]; then
        if [ "$RAISIM_ON" == "ON" ]; then
            for erpon in true false
            do
                ../sim/raiSim/benchmark/RaiElastic666Benchmark \
                --nogui --erp-on=$erpon --dt=$dt --csv=$csv_file
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
				./bulletSim/bulletElastic666Benchmark \
				--nogui --erp-on=$erpon --dt=$dt --csv=$csv_file
			done
        else
            echo "bulletsim is not built. turn on BENCHMARK_BULLETSIM option in cmake"
        fi
    fi

    # ode sim
    if [ "$test_ode" == 'ON' ]; then
        if [ "$ODESIM_ON" == "ON" ] ; then
            for solver in std quick
            do
                for erpon in true false
                do
                    ../sim/odeSim/benchmark/OdeElastic666Benchmark \
                    --nogui --erp-on=$erpon --dt=$dt --solver=$solver --csv=$csv_file
                done
            done
        else
            echo "odesim is not built. turn on BENCHMARK_ODESIM option in cmake"
        fi
    fi

    # mujoco sim
    if [ "$test_mjc" == 'ON' ]; then
        echo "elastic 666 test is not available for mujoco."
    fi

    # dart sim
    if [ "$test_dart" == 'ON' ]; then
        if [ "$DARTSIM_ON" == "ON" ] ; then
            for solver in dantzig pgs
            do
                # note dart has no erp
                ../sim/dartSim/benchmark/DartElastic666Benchmark \
                --nogui --dt=$dt --solver=$solver --csv=$csv_file
            done
        else
            echo "dartsim is not built. turn on BENCHMARK_DARTSIM option in cmake"
        fi
    fi
done
