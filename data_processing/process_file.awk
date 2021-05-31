#!/usr/bin/awk -f
######################################################
### 
### USAGE: awk -f THIS_FILE FILE_TO_PROCESS
###
### The file to process has the output of a DirtMP 
### planner. Looks for lines starting with "[PLANNER]"
### and iterates over it to find the desired values 
### first/last {cost,time,iter}
### Any other line is ignored.
######################################################


# Updated the variables being tracked...
function update_vars(strs){
	if (strs[1] == "cost")
	{
		cost = strs[2]	
		sln_found=1
	}
	else if (strs[1] == "time")
	{
		time = strs[2]
	}
	else if (strs[1] == "iter")
	{
		iter = strs[2]
	}
	else if (strs[1] == "nodes")
	{
		node = strs[2]
	}
	
}

function find_params(strs)
{
	if (strs[1] == "start_state")
	{
		start_state = strs[2]
	}
	else if (strs[1] == "blossom")
	{
		blossom_found = strs[2]
	}
	else if (strs[1] == "sac_her")
	{
		sac_her_found = strs[2]
	}
	else if (strs[1] == "RESELECT_RANDOM")
	{
		RESELECT_RANDOM_found = strs[2]
		# printf("%s\n", RESELECT_RANDOM_found)
	}
	# printf "str: %s\n", strs[1]
}

function reset_vars()
{
	cost = 0
	time = 0
	iter = 0
	node = 0

	first_cost_tmp=0
	first_time_tmp=0
	first_iter_tmp=0
	first_node_tmp=0

	blossom_found = 0
	RESELECT_RANDOM_found = 0
	sac_her_found = 0
}

BEGIN {


PREC=100

first_cost = 0
last_cost = 0

first_time=0
last_time=0

first_iters=0
last_iters=0

first_node=0
last_node=0

cost = 0
time = 0
iter = 0
node = 0

first = 1
sln_found = 0
solutions = 0

blossom_found=0
blossom=1
RESELECT_RANDOM="true"
RESELECT_RANDOM_found=-1
sac_her="true"
sac_her_found=-1
start_state=""
	# for (i = 1; i < ARGC; i++) 
	# {
	# 	printf("ARGV[%d]:%s\n", i, ARGV[i])
 #        if (ARGV[i] ~ /blossom=[0-9]*/)
 #        {
 #            blossom = substr(ARGV[i], index(ARGV[i], "=")+1)
 #        }
 #        else if (ARGV[i] ~ /RESELECT_RANDOM=("true"|"false")/)
 #        {
 #            RESELECT_RANDOM = substr(ARGV[i], index(ARGV[i], "=")+1)
 #        }
 #        else if (ARGV[i] ~ /sac_her=("true"|"false")/)
 #        {
 #            sac_her = substr(ARGV[i], index(ARGV[i], "=")+1)
 #        }
 #    }
}

{
	# Match if first word is "[PLANNER]" e.g.: "[dirt]"
	# if ($0 ~ /\[[a-zA-Z_]*\] Found new goal: .*/)
	# [dirt] Found new goal
	if ($0 ~ /\[[a-zA-Z_]*\] Found new goal: .*/)
	{

		# printf "%s ",start_state, $5
		split($5, aux, ",")
		# for (i in aux) 
		# {
		# 	printf "%s ", aux[i]
		# }
		# printf "\n"
		# # Iterate over the line to split the values
		for (i = 1; i <= NF; i++) 
        {
        	# split string like "cost:100" into aux={cost,100}
			split($i,aux,":") 
			# Update the appropiate variables
			update_vars(aux)
        }
  #   	# Is this the first solution?
		if (first)
		{
			# Update first vars values
			first_cost_tmp = cost
			first_time_tmp = time 
			first_iter_tmp = iter
			first_node_tmp = node
			first = 0
		}
	}
	# Assuming that the last line printed by the program is "End of program"
	else if ($0=="End of program" || $0=="End_of_program")
	{
		# Update the last variables
		first = 1
			# printf("sln_found: %d\n", sln_found)
			# printf("RESELECT_RANDOM_found: %d\n", RESELECT_RANDOM_found)
			# printf("sac_her_found: %d\n", sac_her_found)

		if (sln_found && 
			( blossom == 0 || blossom_found == blossom ) &&
			( RESELECT_RANDOM_found == RESELECT_RANDOM ) &&
			( sac_her_found == sac_her ) )
		{
			first_cost += first_cost_tmp
			first_time += first_time_tmp 
			first_iter += first_iter_tmp
			first_node += first_node_tmp

			last_cost+=cost
			last_time+=time
			last_iter+=iter
			last_node+=node

			solutions += 1
			sln_found = 0
			# printf("Blossom1: %d\n", blossom)
			# printf("solutions: %d\n", solutions)
			# printf("blossom_found: %d\n", blossom_found)

		}
		reset_vars()
	}
	else
	{
		# split string like "cost:100" into aux={cost,100}
		split($0,aux,":") 
		# Find and update the appropiate variables
		find_params(aux)
	}

}

END {
printf("Blossom: %d\n", blossom)
printf("RESELECT_RANDOM: %d\n", RESELECT_RANDOM)
printf("sac_her: %d\n", sac_her)
printf("solutions: %d\n", solutions)

# printf("Tot. Solutions: %d\n", solutions)
# printf("Avg. FirstTime: %.4f\n", first_time/ solutions)
# printf("Avg. FirstCost: %.4f\n", first_cost/ solutions)
# printf("Avg. FirstIter: %.4f\n", first_iter/ solutions)
# printf("Avg. Firstnode: %.4f\n", first_node/ solutions)
# printf("Avg. Last Time: %.4f\n", last_time / solutions)
# printf("Avg. Last Cost: %.4f\n", last_cost / solutions)
# printf("Avg. Last Iter: %.4f\n", last_iter / solutions)
# printf("Avg. Last Node: %.4f\n", last_node / solutions)

printf("%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n", 
	solutions,
	first_time/ solutions,
	first_cost/ solutions,
	first_iter/ solutions,
	first_node/ solutions,
	last_time / solutions,
	last_cost / solutions,
	last_iter / solutions,
	last_node / solutions)
}