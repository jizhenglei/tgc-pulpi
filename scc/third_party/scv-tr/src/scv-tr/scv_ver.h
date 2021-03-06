/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera)
  under one or more contributor license agreements.  See the
  NOTICE file distributed with this work for additional
  information regarding copyright ownership. Accellera licenses
  this file to you under the Apache License, Version 2.0 (the
  "License"); you may not use this file except in compliance
  with the License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing,
  software distributed under the License is distributed on an
  "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
  KIND, either express or implied.  See the License for the
  specific language governing permissions and limitations
  under the License.

 *****************************************************************************/

/*****************************************************************************

  scv_ver.h -- Version and copyright information.

  Original Author: Jerome Cornet, STMicroelectronics

 NO AUTOMATIC CHANGE LOG IS GENERATED, EXPLICIT CHANGE LOG AT END OF FILE
 *****************************************************************************/

#ifndef SCV_TR_VER_H
#define SCV_TR_VER_H

#define SCV_TR_SHORT_RELEASE_DATE 20140417

#define SCV_TR_VERSION_ORIGINATOR "Accellera"
#define SCV_TR_VERSION_MAJOR 2
#define SCV_TR_VERSION_MINOR 0
#define SCV_TR_VERSION_PATCH 0
#define SCV_TR_IS_PRERELEASE 0

// token stringification

#define SCV_TR_STRINGIFY_HELPER_(Arg) SCV_TR_STRINGIFY_HELPER_DEFERRED_(Arg)
#define SCV_TR_STRINGIFY_HELPER_DEFERRED_(Arg) SCV_TR_STRINGIFY_HELPER_MORE_DEFERRED_(Arg)
#define SCV_TR_STRINGIFY_HELPER_MORE_DEFERRED_(Arg) #Arg

#define SCV_TR_VERSION_RELEASE_DATE SCV_TR_STRINGIFY_HELPER_(SCV_TR_SHORT_RELEASE_DATE)

#define SCV_TR_VERSION_PRERELEASE "" // nothing
#define SCV_TR_VERSION                                                                                                 \
    SCV_TR_STRINGIFY_HELPER_(SCV_TR_VERSION_MAJOR.SCV_TR_VERSION_MINOR.SCV_TR_VERSION_PATCH)                           \
    "-" SCV_TR_VERSION_ORIGINATOR

#endif
