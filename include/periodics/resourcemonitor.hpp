#ifndef RESOURCEMONITOR_HPP
#define RESOURCEMONITOR_HPP

// TODO: Add your code here

#include <mbed.h>
#include <drivers/serialtxbroker.hpp>
#include <utils/task.hpp>
#include "mbed_stats.h"
#include <brain/globalsv.hpp>
#include <chrono>

namespace periodics
{
   /**
    * @brief Class resourcemonitor
    *
    */
    class CResourcemonitor : public utils::CTask
    {
        public:
            /* Construnctor */
            CResourcemonitor(
                std::chrono::milliseconds f_period,
                drivers::CSerialTxBroker& f_serialBroker
            );
            /* Destructor */
            ~CResourcemonitor();

            void serialCallbackRESMONCommand(char const * a, char * b);

        private:
            /* private variables & method member */
            virtual void    _run();

            drivers::CSerialTxBroker& m_serialBroker;

            bool m_isActive;
    }; // class CResourcemonitor
}; // namespace drivers

#endif // RESOURCEMONITOR_HPP
