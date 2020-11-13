/*============================================================================
  ==============================================================================

  data_collection_lib.h

  ==============================================================================
  Remarks:

  This is a lib for running an additional thread that communicates
  with a data collection process running on the same machine.

  ============================================================================*/

#ifndef _data_collection_lib_
#define _data_collection_lib_

#ifdef __cplusplus
extern "C" {
#endif

int  initDataCollection(void);
void triggerPoseDeltaPrediction(void);
void triggerDataCollection(void);
int  checkPoseDeltaPrediction(void);
int  checkDataCollection(void);


#ifdef __cplusplus
}
#endif

#endif // _data_collection_lib_
