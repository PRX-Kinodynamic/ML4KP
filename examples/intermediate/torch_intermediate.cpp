#ifndef TORCH_NOT_BUILT
#include <torch/torch.h>
#include <torch/script.h>
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/timer.hpp"

using namespace prx;
// TODO: Check that this file runs and makes sense
// TODO: Should we add the networks to the repo?

void compare_outputs(std::vector<std::vector<double>> v1, std::vector<std::vector<double>> v2, const int n)
{
	double norm1, norm2;
	for (int i = 0; i < n; i++)
	{
		norm1 = vector_norm(v1.at(i));
		norm2 = vector_norm(v2.at(i));
		if (std::fabs(norm1 - norm2) < PRX_EPSILON)
		{
			std::cout << "Conflict! v1 = " << v1.at(i).at(0) << " " << v1.at(i).at(1) << " v2 = " <<
											v2.at(i).at(0) << " " << v2.at(i).at(1) << std::endl;
		}
	}
}

int main(int argc, char* argv[])
{
	try
	{	
		init_random(101193);
		torch::manual_seed(123456);
		torch::NoGradGuard no_grad;

		const unsigned num_trials = int(1e4);
		const unsigned num_predictions = 5;
		prx::timer_t timer;

		std::string network_path;
		// Uncomment below line for second order.
		// #define SECOND_ORDER
		#ifndef SECOND_ORDER
		network_path = lib_path + "resources/networks/FirstOrderTraced.pt";
    	#else
    	network_path = lib_path + "resources/networks/SecondOrderTraced.pt";
    	#endif

        torch::jit::script::Module controller;
        torch::Device device(torch::kCPU);

        try
        {
            controller = torch::jit::load(network_path,device);
        }
        catch(const c10::Error& e)
        {
            std::cerr << "Error loading the model\n";
            return -1;
        }

        double time_taken_single, time_taken_multiple;
        at::Tensor controller_input_single, controller_input_multiple;
        time_taken_single = time_taken_multiple = 0.0;
    	std::vector<torch::jit::IValue> controller_inputs_single, controller_inputs_multiple;
    	std::vector<std::vector<double>> outputs_single, outputs_multiple;

        for (int i = 0; i < num_trials; i++)
        {
        	controller_inputs_single.clear();
			controller_input_single = torch::zeros({num_predictions,5}, device);

        	for (int idx = 0; idx < num_predictions; idx++)
        	{
				controller_inputs_multiple.clear();

        		#ifndef SECOND_ORDER
        		controller_input_multiple = torch::zeros({1,5}, device);
        		controller_input_multiple[0][2] = uniform_random(-PRX_PI,PRX_PI);
        		controller_input_multiple[0][3] = uniform_random(-5.0,5.0);
        		controller_input_multiple[0][4] = uniform_random(-5.0,5.0);

				controller_input_single[idx][2] = controller_input_multiple[0][2];
				controller_input_single[idx][3] = controller_input_multiple[0][3];
				controller_input_single[idx][4] = controller_input_multiple[0][4];
        		#else
        		controller_input_multiple = torch::zeros({1,7}, device);
        		controller_input_multiple[0][2] = uniform_random(-PRX_PI,PRX_PI);
        		controller_input_multiple[0][3] = uniform_random(-0.7,0.7);
        		controller_input_multiple[0][4] = uniform_random(-0.7,0.7);
        		controller_input_multiple[0][5] = uniform_random(-5.0,5.0);
        		controller_input_multiple[0][6] = uniform_random(-5.0,5.0);

				controller_input_single[idx][2] = controller_input_multiple[0][2];
				controller_input_single[idx][3] = controller_input_multiple[0][3];
				controller_input_single[idx][4] = controller_input_multiple[0][4];
				controller_input_single[idx][5] = controller_input_multiple[0][5];
				controller_input_single[idx][6] = controller_input_multiple[0][6];
        		#endif

        		controller_inputs_multiple.push_back(controller_input_multiple);
        		timer.reset();
        		at::Tensor controller_output = controller.forward(controller_inputs_multiple).toTensor();
        		time_taken_multiple += timer.measure();
        		std::vector<double> out = {controller_output[0][0].item<double>(),controller_output[0][1].item<double>()};
        		outputs_multiple.push_back(out);
        	}

			controller_inputs_single.push_back(controller_input_single);
        	timer.reset();
        	at::Tensor controller_output = controller.forward(controller_inputs_single).toTensor();
    		time_taken_single += timer.measure();

    		for (int idx = 0; idx < num_predictions; idx++)
    		{
    			std::vector<double> out = {controller_output[idx][0].item<double>(),controller_output[idx][1].item<double>()};
        		outputs_single.push_back(out);
    		}

    		compare_outputs(outputs_single,outputs_multiple,num_predictions);
        }
        std::cout << "Average time taken for single forward = " << time_taken_single/num_trials << std::endl;
        std::cout << "Average time taken for multiple forward = " << time_taken_multiple/num_trials << std::endl;
	}
	catch(const prx_assert_t& e)
    {
        std::cout << e.get_message() << '\n';
    }
}
#else
int main(int argc, char* argv[])
{}
#endif
