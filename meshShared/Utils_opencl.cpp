#include "stdafx.h"
#include "Utils_opencl.h"
#include <boost/compute.hpp>
#include "Utils_time.h"
#include "Utils_stdvector.h"

using namespace boost;

namespace utils
{
    namespace opencl
    {
         void sort(std::vector<float>& v)
        {
            // get the default compute device
            compute::device gpu = compute::system::default_device();

            // create a compute context and command queue
            compute::context ctx(gpu);
            compute::command_queue queue(ctx, gpu);

            // generate random numbers on the host
            //std::vector<float> host_vector(1000000);
            //std::generate(host_vector.begin(), host_vector.end(), rand);

            //
            // v0 - manually copy to device and from device
            //

            //// create vector on the device
            //compute::vector<D> device_vector(v.size(), ctx);

            //// copy data to the device
            //compute::copy(
            //    v.begin(), v.end(), device_vector.begin(), queue
            //);

            //// sort data on the device
            //compute::sort(
            //    device_vector.begin(), device_vector.end(), queue
            //);

            //// copy data back to the host
            //compute::copy(
            //    device_vector.begin(), device_vector.end(), v.begin(), queue
            //);


            //
            // v1 - use shorcut
            //
            boost::compute::sort(v.begin(), v.end(), queue);
        }

         compute::command_queue prepare_gpu_context()
         {
             // get the default compute device
             compute::device gpu = compute::system::default_device();

             // create a compute context and command queue
             compute::context ctx(gpu);
             compute::command_queue queue(ctx, gpu);
             return queue;
         }


        void printVector(string title, const std::vector<float>& v)
        {
            cout << title << "   " << endl;;
            for (int i = 0; i < v.size(); i++)
            {
                if (i != 0)
                {
                    cout << ", ";
                }
                cout << v[i];
            }
            cout << endl;
        }

        void testSort()
        {
            cout << "------------------------------------------------" << endl;
            cout << "     TESTING  OPENCL  SORT" << endl;
            cout << "------------------------------------------------" << endl;
            std::vector<float> v({ 0,4,1,2,3,5,8,6,4,3,2,6,9,7,6,4,2,1,5,3,8,6,4,9,7,0,8,9,7,3,67,3,5,75,23,54,13,7,35,23,98,54,23,7,43,11,9,87,2 });
            printVector("unsorted", v);
            utils::opencl::sort(v);
            printVector("sorted", v);

            // generate random numbers on the host
            int count = 1024 * 1024;
            cout << "generating random vector with " << count << " elements...";;
            auto t = utils::time::Now();
            std::vector<int> v2intrand(count);
            std::generate(v2intrand.begin(), v2intrand.end(), rand);
            std::vector<float> vCPU(count);
            for (int i = 0; i < v2intrand.size(); i++) vCPU[i] = v2intrand[i];
            std::vector<float> vGPU_cold(vCPU);
            std::vector<float> vGPU_hot(vCPU);
            cout << " done in " << utils::time::ElapsedSecondsStr(t) << endl;

            cout << "sorting on CPU... ";
            t = utils::time::Now();
            utils::stdvector::sort(vCPU);
            cout << (is_sorted(vCPU.begin(), vCPU.end()) ? " sorted." : " failed to sort!") << " done in " << utils::time::ElapsedSecondsStr(t) << endl;

            cout << "sorting on GPU (cold)... ";
            t = utils::time::Now();
            utils::opencl::sort(vGPU_cold);
            cout << (is_sorted(vGPU_cold.begin(), vGPU_cold.end()) ? " sorted." : " failed to sort!") << " done in " << utils::time::ElapsedSecondsStr(t) << endl;

            compute::command_queue queue = prepare_gpu_context();
            cout << "sorting on GPU (hot)... ";
            t = utils::time::Now();
            boost::compute::sort(vGPU_hot.begin(), vGPU_hot.end(), queue);
            cout << (is_sorted(vGPU_hot.begin(), vGPU_hot.end()) ? " sorted." : " failed to sort!") << " done in " << utils::time::ElapsedSecondsStr(t) << endl;



            cout << "------------------------------------------------" << endl;
        }
    }
}