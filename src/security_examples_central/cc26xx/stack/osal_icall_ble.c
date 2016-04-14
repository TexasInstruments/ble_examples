/******************************************************************************

 @file  osal_icall_ble.c

 @brief This file contains function that allows user setup tasks

 Group: WCS, BTS
 $Target Device: DEVICES $

 ******************************************************************************
 $License: BSD3 2013 $
 ******************************************************************************
 $Release Name: PACKAGE NAME $
 $Release Date: PACKAGE RELEASE DATE $
 *****************************************************************************/

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include <icall.h>
#include "hal_types.h"
#include "hal_mcu.h"
#include "osal.h"
#include "osal_tasks.h"
#include "osal_snv.h"


/* LL */
#include "ll.h"

#if defined ( OSAL_CBTIMER_NUM_TASKS )
  #include "osal_cbtimer.h"
#endif

/* L2CAP */
#include "l2cap.h"

/* gap */
#include "gap.h"

#if defined ( GAP_BOND_MGR )
  #include "gapbondmgr.h"
#endif

/* GATT */
#include "gatt.h"

/* Application */
#include "hci_tl.h"

#include "gattservapp.h"

#include "ble_user_config.h"
#include "ble_dispatch.h"

/*********************************************************************
 * GLOBAL VARIABLES
 */

// The order in this table must be identical to the task initialization calls below in osalInitTask.
const pTaskEventHandlerFn tasksArr[] =
{
  LL_ProcessEvent,                                                  // task 0
  HCI_ProcessEvent,                                                 // task 1
#if defined ( OSAL_CBTIMER_NUM_TASKS )
  OSAL_CBTIMER_PROCESS_EVENT( osal_CbTimerProcessEvent ),           // task 2
#endif
  L2CAP_ProcessEvent,                                               // task 3
  GAP_ProcessEvent,                                                 // task 4
  SM_ProcessEvent,                                                  // task 5
  GATT_ProcessEvent,                                                // task 6
  GATTServApp_ProcessEvent,                                         // task 7
#if defined ( GAP_BOND_MGR )
  GAPBondMgr_ProcessEvent,                                          // task 8
#endif
  bleDispatch_ProcessEvent                                          // task 9
};

const uint8 tasksCnt = sizeof( tasksArr ) / sizeof( tasksArr[0] );
uint16 *tasksEvents;

/*********************************************************************
 * FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      osalInitTasks
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
void osalInitTasks( void )
{
  ICall_EntityID entity;
  ICall_Semaphore sem;
  uint8 taskID = 0;
  uint8 i;

  tasksEvents = (uint16 *)osal_mem_alloc( sizeof( uint16 ) * tasksCnt);
  osal_memset( tasksEvents, 0, (sizeof( uint16 ) * tasksCnt));

  /* LL Task */
  LL_Init( taskID++ );

  /* HCI Task */
  HCI_Init( taskID++ );

#if defined ( OSAL_CBTIMER_NUM_TASKS )
  /* Callback Timer Tasks */
  osal_CbTimerInit( taskID );
  taskID += OSAL_CBTIMER_NUM_TASKS;
#endif

  /* L2CAP Task */
  L2CAP_Init( taskID++ );

  /* GAP Task */
  GAP_Init( taskID++ );

	  /* SM Task */
  SM_Init( taskID++ );
	
  /* GATT Task */
  GATT_Init( taskID++ );
 
  /* GATT Server App Task */
  GATTServApp_Init( taskID++ );
    
#if defined ( GAP_BOND_MGR )
  /* Bond Manager Task */
  GAPBondMgr_Init( taskID++ );
#endif
  
  /* ICall BLE Dispatcher Task */
  bleDispatch_Init( taskID );

  // ICall enrollment
  /* Enroll the service that this stack represents */
  ICall_enrollService(ICALL_SERVICE_CLASS_BLE, NULL, &entity, &sem);

  /* Enroll the obtained dispatcher entity and OSAL task ID of HCI Ext App
   * to OSAL so that OSAL can route the dispatcher message into
   * the appropriate OSAL task.
   */
  osal_enroll_dispatchid(taskID, entity);

  /* Register all other OSAL tasks to use the registered dispatcher entity
   * ID as the source of dispatcher messages, even though the other OSAL
   * tasks didn't register themselves to receive messages from application.
   */
  for (i = 0; i < taskID; i++)
  {
    osal_enroll_senderid(i, entity);
  }
}

/**
 * Main entry function for the stack image
 */
int stack_main( void *arg )
{
  /* User reconfiguration of BLE Controller and Host variables */
  setBleUserConfig( (bleUserCfg_t *)arg );
  
  /* Establish OSAL for a stack service that requires accompanying
   * messaging service */
  if (ICall_enrollService(ICALL_SERVICE_CLASS_BLE_MSG,
                          (ICall_ServiceFunc) osal_service_entry,
                          &osal_entity, &osal_semaphore) !=
      ICALL_ERRNO_SUCCESS)
  {
    /* abort */
    ICall_abort();
  }

  halIntState_t state;
  HAL_ENTER_CRITICAL_SECTION(state);
  
  // Turn off interrupts
  //osal_int_disable( INTS_ALL );

  // Initialize NV System
  osal_snv_init( );

  // Initialize the operating system
  osal_init_system();

  // Allow interrupts
  //osal_int_enable( INTS_ALL );
  HAL_EXIT_CRITICAL_SECTION(state);

  osal_start_system(); // No Return from here

  return 0;  // Shouldn't get here.
}

/*********************************************************************
*********************************************************************/
