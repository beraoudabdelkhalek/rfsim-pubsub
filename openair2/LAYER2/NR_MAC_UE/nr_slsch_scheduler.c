/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */

/* \file        nr_slsch_scheduler.c
 * \brief       Routines for UE SLSCH scheduling
 * \author      R. Knopp 
 * \date        Aug. 2023
 * \version     0.1
 * \company     EURECOM 
 * \email       raymond.knopp@eurecom.fr 
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include <common/utils/nr/nr_common.h>

#include "NR_MAC_COMMON/nr_mac.h"
#include "NR_MAC_COMMON/nr_mac_common.h"
#include "NR_MAC_UE/mac_proto.h"
#include "NR_MAC_UE/mac_extern.h"
#include "NR_MAC_UE/nr_ue_sci.h"
#include <executables/nr-uesoftmodem.h>

bool nr_schedule_slsch(NR_UE_MAC_INST_t *mac, int frameP,int slotP, nr_sci_pdu_t *sci_pdu,nr_sci_pdu_t *sci2_pdu,uint8_t *slsch_pdu,nr_sci_format_t format2, uint16_t *slsch_pdu_length_max) {

  mac_rlc_status_resp_t rlc_status = mac_rlc_status_ind(0, mac->src_id, 0, frameP, slotP, ENB_FLAG_NO, MBMS_FLAG_NO, 4, 0, 0);

  sl_nr_ue_mac_params_t *sl_mac = mac->SL_MAC_PARAMS;
  uint8_t mu = sl_mac->sl_phy_config.sl_config_req.sl_bwp_config.sl_scs;
  uint8_t slots_per_frame = nr_slots_per_frame[mu];

  uint8_t psfch_period = 0;
  if (mac->sl_tx_res_pool->sl_PSFCH_Config_r16 && mac->sl_tx_res_pool->sl_PSFCH_Config_r16->choice.setup->sl_PSFCH_Period_r16)
    psfch_period = *mac->sl_tx_res_pool->sl_PSFCH_Config_r16->choice.setup->sl_PSFCH_Period_r16;
  *slsch_pdu_length_max = 0;
  bool csi_acq = !mac->SL_MAC_PARAMS->sl_CSI_Acquisition;
  bool csi_req_slot = !((slots_per_frame * frameP + slotP - sl_mac->slot_offset) % sl_mac->slot_periodicity);
  if (rlc_status.bytes_in_buffer > 0 || mac->sci_pdu_rx.harq_feedback || (csi_acq && csi_req_slot) || mac->sl_csi_report) {
     // Fill SCI1A
     sci_pdu->priority = 0;
     sci_pdu->frequency_resource_assignment.val = 0;
     sci_pdu->time_resource_assignment.val = 0;
     sci_pdu->resource_reservation_period.val = 0;
     sci_pdu->dmrs_pattern.val = 0;
     sci_pdu->second_stage_sci_format = 0;
     sci_pdu->number_of_dmrs_port = 0;
     sci_pdu->mcs = 9;
     sci_pdu->additional_mcs.val = 0;
     sci_pdu->psfch_overhead.val = 0;
     sci_pdu->reserved.val = 0;
     sci_pdu->conflict_information_receiver.val = 0;
     sci_pdu->beta_offset_indicator = 0;

     // Fill SCI2A
     sci2_pdu->harq_pid = 0;
     sci2_pdu->ndi = (1 - sci2_pdu->ndi) & 1;
     sci2_pdu->rv_index = 0;
     sci2_pdu->source_id = get_softmodem_params()->node_number;
     sci2_pdu->dest_id = 0xabcd;
     sci2_pdu->harq_feedback = psfch_period ? 1 : 0;
     sci2_pdu->cast_type = 1;
     if (format2 == NR_SL_SCI_FORMAT_2C || format2 == NR_SL_SCI_FORMAT_2A) {
       sci2_pdu->csi_req = (csi_acq && csi_req_slot) ? 1 : 0;
       LOG_D(NR_MAC, "%4d.%2d Setting sci2_pdu->csi_req %d\n", frameP, slotP, sci2_pdu->csi_req);
     }
     if (format2 == NR_SL_SCI_FORMAT_2B)
       sci2_pdu->zone_id = 0;
     // Fill in for R17: communication_range
     sci2_pdu->communication_range.val = 0;
     if (format2 == NR_SL_SCI_FORMAT_2C) {
       sci2_pdu->providing_req_ind = 0;
       // Fill in for R17 : resource combinations
       sci2_pdu->resource_combinations.val = 0;
       sci2_pdu->first_resource_location = 0;
       // Fill in for R17 : reference_slot_location
       sci2_pdu->reference_slot_location.val = 0;
       sci2_pdu->resource_set_type = 0;
       // Fill in for R17 : lowest_subchannel_indices
       sci2_pdu->lowest_subchannel_indices.val = 0;
     }
     // Set SLSCH
     *slsch_pdu_length_max = rlc_status.bytes_in_buffer;
     return true;
   }
   return false;
}

