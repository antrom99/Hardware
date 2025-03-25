/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   exceptions.h
 * Author:  Giovanni Longobardi, Giovanni Mignone
 * Org.:    UNISA
 * Date:    Dec 10, 2020
 *
 * This module masks the ros::Exception class in the Exception
 * class of this namespace/package.
 *
 * -------------------------------------------------------------------
 */

#ifndef INCLUDE_C4G_HARDWARE_INTERFACE_EXCEPTIONS_H_
#define INCLUDE_C4G_HARDWARE_INTERFACE_EXCEPTIONS_H_

#include <ros/exception.h>

namespace c4g_hardware_interface
{
typedef ros::Exception Exception;
}

#endif /* INCLUDE_C4G_HARDWARE_INTERFACE_EXCEPTIONS_H_ */