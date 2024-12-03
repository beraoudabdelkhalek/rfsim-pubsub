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

#define _GNU_SOURCE

#include "PHY/defs_nr_UE.h"
#include <openair1/PHY/TOOLS/phy_scope_interface.h>
#include "openair1/PHY/NR_TRANSPORT/nr_ulsch.h"
#include "openair1/PHY/NR_TRANSPORT/nr_transport_proto.h"
#include "common/utils/LOG/log.h"
#include "common/utils/utils.h"
#include "common/utils/LOG/vcd_signal_dumper.h"
#include "UTIL/OPT/opt.h"
#include "intertask_interface.h"
#include "T.h"
#include "PHY/MODULATION/modulation_UE.h"
#include "PHY/NR_UE_ESTIMATION/nr_estimation.h"
#include "PHY/NR_UE_TRANSPORT/nr_transport_proto_ue.h"
#include "executables/nr-uesoftmodem.h"
#include "common/utils/colors.h"

void nr_fill_indication(PHY_VARS_gNB *gNB, int frame, int slot_rx, int ULSCH_id, uint8_t harq_pid, uint8_t crc_flag, int dtx_flag) {
  AssertFatal(1==0,"Should never get here\n"); 
}
NR_gNB_PHY_STATS_t *get_phy_stats(PHY_VARS_gNB *gNB, uint16_t rnti) {
  return(NULL); 
}
void nr_fill_sl_indication(nr_sidelink_indication_t *sl_ind,
                           sl_nr_rx_indication_t *rx_ind,
                           sl_nr_sci_indication_t *sci_ind,
                           UE_nr_rxtx_proc_t *proc,
                           PHY_VARS_NR_UE *ue,
                           void *phy_data)
{
  memset((void*)sl_ind, 0, sizeof(nr_sidelink_indication_t));

  sl_ind->gNB_index = proc->gNB_id;
  sl_ind->module_id = ue->Mod_id;
  sl_ind->cc_id     = ue->CC_id;
  sl_ind->frame_rx  = proc->frame_rx;
  sl_ind->slot_rx   = proc->nr_slot_rx;
  sl_ind->frame_tx  = proc->frame_tx;
  sl_ind->slot_tx   = proc->nr_slot_tx;
  sl_ind->phy_data  = phy_data;
  sl_ind->slot_type = SIDELINK_SLOT_TYPE_RX;

  if (rx_ind) {
    sl_ind->rx_ind = rx_ind; //  hang on rx_ind instance
    sl_ind->sci_ind = NULL;
  } 
  if (sci_ind) {
    sl_ind->rx_ind = NULL;
    sl_ind->sci_ind = sci_ind;
  }
}

void nr_fill_sl_rx_indication(sl_nr_rx_indication_t *rx_ind,
                              uint8_t pdu_type,
                              PHY_VARS_NR_UE *ue,
                              uint16_t n_pdus,
                              UE_nr_rxtx_proc_t *proc,
                              void *typeSpecific,
                              uint16_t rx_slss_id)
{

  if (n_pdus > 1){
    LOG_E(PHY, "In %s: multiple number of SL PDUs not supported yet...\n", __FUNCTION__);
  }

  sl_nr_ue_phy_params_t *sl_phy_params = &ue->SL_UE_PHY_PARAMS;

  switch (pdu_type){
    case SL_NR_RX_PDU_TYPE_SLSCH: {
        sl_nr_slsch_pdu_t *rx_slsch_pdu = &rx_ind->rx_indication_body[n_pdus - 1].rx_slsch_pdu;
        slsch_status_t *slsch_status = (slsch_status_t *)typeSpecific;
        rx_slsch_pdu->pdu        = slsch_status->rdata->ulsch_harq->b;
        rx_slsch_pdu->pdu_length = slsch_status->rdata->ulsch_harq->TBS;
        rx_slsch_pdu->harq_pid   = slsch_status->rdata->harq_pid;
        rx_slsch_pdu->ack_nack   = (slsch_status->rxok==true) ? 1 : 0;

        if (slsch_status->rxok==true) ue->SL_UE_PHY_PARAMS.pssch.rx_ok++;
        else                          ue->SL_UE_PHY_PARAMS.pssch.rx_errors[0]++;
      }
      break;
    case FAPI_NR_RX_PDU_TYPE_SSB: {
        sl_nr_ssb_pdu_t *ssb_pdu = &rx_ind->rx_indication_body[n_pdus - 1].ssb_pdu;
        if(typeSpecific) {
          uint8_t *psbch_decoded_output = (uint8_t *)typeSpecific;
          memcpy(ssb_pdu->psbch_payload, psbch_decoded_output, sizeof(4));//4 bytes of PSBCH payload bytes
          ssb_pdu->rsrp_dbm = sl_phy_params->psbch.rsrp_dBm_per_RE;
          ssb_pdu->rx_slss_id = rx_slss_id;
          ssb_pdu->decode_status = true;
          LOG_D(PHY, "SL-IND: SSB to MAC. rsrp:%d, slssid:%d, payload:%x\n",
                                                    ssb_pdu->rsrp_dbm,ssb_pdu->rx_slss_id,
                                                    *((uint32_t *)(ssb_pdu->psbch_payload)) );
        }
        else
          ssb_pdu->decode_status = false;
      }
      break;
    default:
      break;
  }

  rx_ind->rx_indication_body[n_pdus -1].pdu_type = pdu_type;
  rx_ind->number_pdus = n_pdus;

}

extern int dmrs_pscch_mask[2];
int nr_slsch_procedures(PHY_VARS_NR_UE *ue, int frame_rx, int slot_rx, int SLSCH_id, UE_nr_rxtx_proc_t *proc, nr_phy_data_t *phy_data, bool is_csi_rs_slot) {


  sl_nr_ue_phy_params_t *sl_phy_params = &ue->SL_UE_PHY_PARAMS;
  NR_DL_FRAME_PARMS *fp = &sl_phy_params->sl_frame_params;
  sl_nr_rx_config_pssch_pdu_t *slsch_pdu = &phy_data->nr_sl_pssch_pdu; //ue->slsch[SLSCH_id].harq_process->slsch_pdu;
  sl_nr_rx_config_pssch_sci_pdu_t *pssch_pdu = &phy_data->nr_sl_pssch_sci_pdu; //ue->slsch[SLSCH_id].harq_process->pssch_pdu;

  uint8_t  freq_density = 0;
  uint8_t  nr_of_rbs = 0;
  if (is_csi_rs_slot) {
    freq_density = ue->csirs_vars[0]->csirs_config_pdu.freq_density;
    nr_of_rbs = ue->csirs_vars[0]->csirs_config_pdu.nr_of_rbs;
    AssertFatal((freq_density == 1) || (nr_of_rbs > 0), "CSI-RS parameters are not properly configured\n");
  }
  int harq_pid = slsch_pdu->harq_pid;
  uint16_t nb_re_dmrs;
  uint16_t start_symbol = 1;
  uint16_t number_symbols = pssch_pdu->pssch_numsym;
  ue->slsch[SLSCH_id].harq_process->harq_to_be_cleared=true;
  uint8_t number_dmrs_symbols = 0;
  for (int l = start_symbol; l < start_symbol + number_symbols; l++)
    number_dmrs_symbols += ((pssch_pdu->dmrs_symbol_position)>>l)&0x01;

  nb_re_dmrs = 6;

  uint32_t rb_size                   = pssch_pdu->num_subch*pssch_pdu->subchannel_size;
  int sci1_dmrs_overlap = pssch_pdu->dmrs_symbol_position & dmrs_pscch_mask[pssch_pdu->pscch_numsym-2];
  int sci2_re = get_NREsci2_2(pssch_pdu->sci2_alpha_times_100,
                              pssch_pdu->sci2_len,
                              pssch_pdu->sci2_beta_offset,
                              pssch_pdu->pssch_numsym,
                              pssch_pdu->pscch_numsym,
                              pssch_pdu->pscch_numrbs,
                              pssch_pdu->l_subch,
                              pssch_pdu->subchannel_size,
			      pssch_pdu->targetCodeRate);
  uint32_t G = nr_get_G_SL(rb_size,
                           number_symbols,
                           nb_re_dmrs,
                           number_dmrs_symbols, // number of dmrs symbols irrespective of single or double symbol dmrs
                           sci1_dmrs_overlap,
			   pssch_pdu->pscch_numsym,
			   pssch_pdu->pscch_numrbs,
			   sci2_re,
                           pssch_pdu->mod_order,
                           pssch_pdu->num_layers);

  G -= is_csi_rs_slot ? nr_of_rbs/freq_density*pssch_pdu->mod_order*pssch_pdu->num_layers : 0;

  AssertFatal(G>0,"G is 0 : rb_size %u, number_symbols %d, nb_re_dmrs %d, number_dmrs_symbols %d, qam_mod_order %u, nrOfLayer %u\n",
	      rb_size,
	      number_symbols,
	      nb_re_dmrs,
	      number_dmrs_symbols, // number of dmrs symbols irrespective of single or double symbol dmrs
	      pssch_pdu->mod_order,
	      pssch_pdu->num_layers);
  LOG_D(NR_PHY,"rb_size %d, number_symbols %d, nb_re_dmrs %d, dmrs symbol positions %d, number_dmrs_symbols %d, qam_mod_order %d, nrOfLayer %d\n",
	rb_size,
	number_symbols,
	nb_re_dmrs,
        pssch_pdu->dmrs_symbol_position,
	number_dmrs_symbols, // number of dmrs symbols irrespective of single or double symbol dmrs
	pssch_pdu->mod_order,
	pssch_pdu->num_layers);

  nr_ulsch_layer_demapping(ue->pssch_vars[SLSCH_id].llr,
                           pssch_pdu->num_layers,
                           pssch_pdu->mod_order,
                           G,
                           ue->pssch_vars[SLSCH_id].llr_layers);

  //for (int g=0;g<G;g++) LOG_I(NR_PHY,"prescrambling_llr[%d] %d\n",g,ue->pssch_vars[SLSCH_id].llr[g]);
  //----------------------------------------------------------
  //------------------- ULSCH unscrambling -------------------
  //----------------------------------------------------------
  //LOG_I(NR_PHY,"SLSCH, unscrambling with Nid %x\n",pssch_pdu->Nid);
  nr_ulsch_unscrambling(ue->pssch_vars[SLSCH_id].llr, G, pssch_pdu->Nid, 1010);
//  for (int g=0;g<32;g++) LOG_I(NR_PHY,"unscrambling_llr[%d] %d\n",g,ue->pssch_vars[SLSCH_id].llr[g]);
  //----------------------------------------------------------
  //--------------------- ULSCH decoding ---------------------
  //----------------------------------------------------------


  nfapi_nr_pusch_pdu_t pusch_pdu;

  pusch_pdu.rb_size = rb_size;
  pusch_pdu.qam_mod_order = pssch_pdu->mod_order;
  pusch_pdu.mcs_index = slsch_pdu->mcs;
  pusch_pdu.nrOfLayers = pssch_pdu->num_layers;
  pusch_pdu.pusch_data.tb_size=slsch_pdu->tb_size;
  uint32_t A = slsch_pdu->tb_size<<3;
  pusch_pdu.target_code_rate=slsch_pdu->target_coderate;
  float Coderate = (float) (slsch_pdu->target_coderate) / 10240.0f;
  pusch_pdu.pusch_data.rv_index=slsch_pdu->rv_index;
  
  if ((A <=292) || ((A<=3824) && (Coderate <= 0.6667)) || Coderate <= 0.25){
    pusch_pdu.maintenance_parms_v3.ldpcBaseGraph=2;
  }
  else{
    pusch_pdu.maintenance_parms_v3.ldpcBaseGraph=1;
  }
  pusch_pdu.maintenance_parms_v3.tbSizeLbrmBytes=slsch_pdu->tbslbrm>>3;

  int nbDecode =
      nr_ulsch_decoding(NULL, ue, SLSCH_id, ue->pssch_vars[SLSCH_id].llr, fp, &pusch_pdu, frame_rx, slot_rx, harq_pid, G,proc,phy_data);
  return nbDecode;
}

void nr_postDecode_slsch(PHY_VARS_NR_UE *UE, notifiedFIFO_elt_t *req,UE_nr_rxtx_proc_t *proc,nr_phy_data_t *phy_data)
{
  ldpcDecode_t *rdata = (ldpcDecode_t*) NotifiedFifoData(req);
  NR_UL_gNB_HARQ_t *slsch_harq = rdata->ulsch_harq;
  NR_gNB_ULSCH_t *slsch = rdata->ulsch;
  int r = rdata->segment_r;
  sl_nr_rx_config_pssch_pdu_t *slsch_pdu = &phy_data->nr_sl_pssch_pdu;//UE->slsch[rdata->ulsch_id].harq_process->slsch_pdu;
  bool decodeSuccess = (rdata->decodeIterations <= rdata->decoderParms.numMaxIter);
  slsch_harq->processedSegments++;
  LOG_D(NR_PHY,
        "processing result of segment: %d, processed %d/%d\n",
        rdata->segment_r,
        slsch_harq->processedSegments,
        rdata->nbSegments);
  if (decodeSuccess) {
    memcpy(slsch_harq->b + rdata->offset, slsch_harq->c[r], rdata->Kr_bytes - (slsch_harq->F >> 3) - ((slsch_harq->C > 1) ? 3 : 0));

  } else {
    LOG_D(NR_PHY, "ULSCH %d in error\n", rdata->ulsch_id);
  }

  //int dumpsig=0;
  // if all segments are done
  if (rdata->nbSegments == slsch_harq->processedSegments) {
    sl_nr_rx_indication_t sl_rx_indication;	  
    nr_sidelink_indication_t sl_indication;	  
    slsch_status_t slsch_status;
    if (!check_abort(&slsch_harq->abort_decode) && !UE->pssch_vars[rdata->ulsch_id].DTX) {
      LOG_D(NR_PHY,
            "[UE] SLSCH: Setting ACK for SFN/SF %d.%d (pid %d, ndi %d, status %d, round %d, TBS %d, Max interation "
            "(all seg) %d)\n",
            slsch->frame,
            slsch->slot,
            rdata->harq_pid,
            slsch_pdu->ndi,
            slsch->active,
            slsch_harq->round,
            slsch_harq->TBS,
            rdata->decodeIterations);
      slsch->active = false;
      slsch_harq->round = 0;
      LOG_D(NR_PHY, "SLSCH received ok \n");
      slsch_status.rdata = rdata;
      slsch_status.rxok = true;
      //dumpsig=1;
    } else {
      LOG_D(NR_PHY,
            "[UE] SLSCH: Setting NAK for SFN/SF %d/%d (pid %d, ndi %d, status %d, round %d, RV %d, prb_start %d, prb_size %d, "
            "TBS %d) r %d\n",
            slsch->frame,
            slsch->slot,
            rdata->harq_pid,
            slsch_pdu->ndi,
            slsch->active,
            slsch_harq->round,
            slsch_harq->ulsch_pdu.pusch_data.rv_index,
            slsch_harq->ulsch_pdu.rb_start,
            slsch_harq->ulsch_pdu.rb_size,
            slsch_harq->TBS,
            r);
      slsch->handled = 1;
      LOG_D(NR_PHY, "SLSCH %d in error\n",rdata->ulsch_id);
      slsch_status.rdata = rdata;
      slsch_status.rxok = false;
      //      dumpsig=1;
    }
    slsch->last_iteration_cnt = rdata->decodeIterations;
    sl_rx_indication.sfn = proc->frame_rx;
    sl_rx_indication.slot = proc->nr_slot_rx;
    nr_fill_sl_rx_indication(&sl_rx_indication,SL_NR_RX_PDU_TYPE_SLSCH,UE,1,proc,(void*)&slsch_status,0);
    nr_fill_sl_indication(&sl_indication,&sl_rx_indication,NULL,proc,UE,phy_data);
    if (UE->if_inst && UE->if_inst->sl_indication)
      UE->if_inst->sl_indication(&sl_indication);
    /*
        if (ulsch_harq->ulsch_pdu.mcs_index == 0 && dumpsig==1) {
          int off = ((ulsch_harq->ulsch_pdu.rb_size&1) == 1)? 4:0;

          LOG_M("rxsigF0.m","rxsF0",&gNB->common_vars.rxdataF[0][(ulsch_harq->slot&3)*gNB->frame_parms.ofdm_symbol_size*gNB->frame_parms.symbols_per_slot],gNB->frame_parms.ofdm_symbol_size*gNB->frame_parms.symbols_per_slot,1,1);
          LOG_M("rxsigF0_ext.m","rxsF0_ext",
                 &gNB->pusch_vars[0].rxdataF_ext[0][ulsch_harq->ulsch_pdu.start_symbol_index*NR_NB_SC_PER_RB *
       ulsch_harq->ulsch_pdu.rb_size],ulsch_harq->ulsch_pdu.nr_of_symbols*(off+(NR_NB_SC_PER_RB *
       ulsch_harq->ulsch_pdu.rb_size)),1,1); LOG_M("chestF0.m","chF0",
                &gNB->pusch_vars[0].ul_ch_estimates[0][ulsch_harq->ulsch_pdu.start_symbol_index*gNB->frame_parms.ofdm_symbol_size],gNB->frame_parms.ofdm_symbol_size,1,1);
          LOG_M("chestF0_ext.m","chF0_ext",
                &gNB->pusch_vars[0]->ul_ch_estimates_ext[0][(ulsch_harq->ulsch_pdu.start_symbol_index+1)*(off+(NR_NB_SC_PER_RB *
       ulsch_harq->ulsch_pdu.rb_size))], (ulsch_harq->ulsch_pdu.nr_of_symbols-1)*(off+(NR_NB_SC_PER_RB *
       ulsch_harq->ulsch_pdu.rb_size)),1,1); LOG_M("rxsigF0_comp.m","rxsF0_comp",
                &gNB->pusch_vars[0].rxdataF_comp[0][ulsch_harq->ulsch_pdu.start_symbol_index*(off+(NR_NB_SC_PER_RB *
       ulsch_harq->ulsch_pdu.rb_size))],ulsch_harq->ulsch_pdu.nr_of_symbols*(off+(NR_NB_SC_PER_RB *
       ulsch_harq->ulsch_pdu.rb_size)),1,1); LOG_M("rxsigF0_llr.m","rxsF0_llr",
                &gNB->pusch_vars[0].llr[0],(ulsch_harq->ulsch_pdu.nr_of_symbols-1)*NR_NB_SC_PER_RB * ulsch_harq->ulsch_pdu.rb_size *
       ulsch_harq->ulsch_pdu.qam_mod_order,1,0); if (gNB->frame_parms.nb_antennas_rx > 1) {

            LOG_M("rxsigF1_ext.m","rxsF0_ext",
                   &gNB->pusch_vars[0].rxdataF_ext[1][ulsch_harq->ulsch_pdu.start_symbol_index*NR_NB_SC_PER_RB *
       ulsch_harq->ulsch_pdu.rb_size],ulsch_harq->ulsch_pdu.nr_of_symbols*(off+(NR_NB_SC_PER_RB *
       ulsch_harq->ulsch_pdu.rb_size)),1,1); LOG_M("chestF1.m","chF1",
                  &gNB->pusch_vars[0].ul_ch_estimates[1][ulsch_harq->ulsch_pdu.start_symbol_index*gNB->frame_parms.ofdm_symbol_size],gNB->frame_parms.ofdm_symbol_size,1,1);
            LOG_M("chestF1_ext.m","chF1_ext",
                  &gNB->pusch_vars[0].ul_ch_estimates_ext[1][(ulsch_harq->ulsch_pdu.start_symbol_index+1)*(off+(NR_NB_SC_PER_RB *
       ulsch_harq->ulsch_pdu.rb_size))], (ulsch_harq->ulsch_pdu.nr_of_symbols-1)*(off+(NR_NB_SC_PER_RB *
       ulsch_harq->ulsch_pdu.rb_size)),1,1); LOG_M("rxsigF1_comp.m","rxsF1_comp",
                  &gNB->pusch_vars[0].rxdataF_comp[1][ulsch_harq->ulsch_pdu.start_symbol_index*(off+(NR_NB_SC_PER_RB *
       ulsch_harq->ulsch_pdu.rb_size))],ulsch_harq->ulsch_pdu.nr_of_symbols*(off+(NR_NB_SC_PER_RB *
       ulsch_harq->ulsch_pdu.rb_size)),1,1);
          }
          exit(-1);

        }
    */
    slsch->last_iteration_cnt = rdata->decodeIterations;
    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_gNB_ULSCH_DECODING,0);
  }
}
static int nr_ue_psbch_procedures(PHY_VARS_NR_UE *ue,
                                  NR_DL_FRAME_PARMS *fp,
                                  UE_nr_rxtx_proc_t *proc,
                                  int estimateSz,
                                  struct complex16 dl_ch_estimates[][estimateSz],
                                  nr_phy_data_t *phy_data,
                                  c16_t rxdataF[][fp->samples_per_slot_wCP])
{

  int ret = 0;
  DevAssert(ue);

  int frame_rx = proc->frame_rx;
  int nr_slot_rx = proc->nr_slot_rx;

  sl_nr_ue_phy_params_t *sl_phy_params = &ue->SL_UE_PHY_PARAMS;
  uint16_t rx_slss_id = sl_phy_params->sl_config.sl_sync_source.rx_slss_id;

  //VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_PSBCH_PROCEDURES, VCD_FUNCTION_IN);

  LOG_D(PHY,"[UE  %d] Frame %d Slot %d, Trying PSBCH (SLSS ID %d)\n",
                        ue->Mod_id,frame_rx,nr_slot_rx,
                        sl_phy_params->sl_config.sl_sync_source.rx_slss_id);

  uint8_t decoded_pdu[4] = {0};
  ret = nr_rx_psbch(ue,
                   proc,
                   estimateSz,
                   dl_ch_estimates,
                   fp,
                   decoded_pdu,
                   rxdataF,
                   sl_phy_params->sl_config.sl_sync_source.rx_slss_id);

  nr_sidelink_indication_t sl_indication;
  sl_nr_rx_indication_t rx_ind = {0};
  uint16_t number_pdus = 1;

  uint8_t *result = NULL;
  if (ret) sl_phy_params->psbch.rx_errors ++;
  else {
    result = decoded_pdu;
    sl_phy_params->psbch.rx_ok ++;
  }

  nr_fill_sl_indication(&sl_indication, &rx_ind, NULL, proc, ue, phy_data);
  nr_fill_sl_rx_indication(&rx_ind, SL_NR_RX_PDU_TYPE_SSB, ue, number_pdus, proc, (void *)result, rx_slss_id);

  if (ue->if_inst && ue->if_inst->sl_indication)
    ue->if_inst->sl_indication(&sl_indication);

  //VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_PSBCH_PROCEDURES, VCD_FUNCTION_OUT);
  return ret;
}



void psbch_pscch_pssch_processing(PHY_VARS_NR_UE *ue,
                                  UE_nr_rxtx_proc_t *proc,
                                  nr_phy_data_t *phy_data) {

  int frame_rx = proc->frame_rx;
  int nr_slot_rx = proc->nr_slot_rx;
  sl_nr_ue_phy_params_t *sl_phy_params = &ue->SL_UE_PHY_PARAMS;
  NR_DL_FRAME_PARMS *fp = &sl_phy_params->sl_frame_params;
  bool is_csi_rs_slot = false;

  //VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_UE_RX_SL, VCD_FUNCTION_IN);
  start_meas(&sl_phy_params->phy_proc_sl_rx);

  LOG_D(PHY," ****** Sidelink RX-Chain for Frame.Slot %d.%d ******  \n",
                                                    frame_rx%1024, nr_slot_rx);

  const uint32_t rxdataF_sz = fp->samples_per_slot_wCP;
  __attribute__ ((aligned(32))) c16_t rxdataF[fp->nb_antennas_rx][rxdataF_sz];

  if ((frame_rx&127) == 0  && nr_slot_rx==19) {
      LOG_I(NR_PHY,"============================================\n");

      LOG_I(NR_PHY,"%s[UE%d] %d:%d PSBCH Stats: TX %d, RX ok %d, RX not ok %d\n",KGRN,
                                                      ue->Mod_id, frame_rx, nr_slot_rx,
                                                      sl_phy_params->psbch.num_psbch_tx,
                                                      sl_phy_params->psbch.rx_ok,
                                                      sl_phy_params->psbch.rx_errors);

      LOG_I(NR_PHY,"%s[UE%d] %d:%d PSCCH Stats: TX %d, RX ok %d\n",KGRN,
                                                      ue->Mod_id, frame_rx, nr_slot_rx,
                                                      sl_phy_params->pscch.num_pscch_tx,
                                                      sl_phy_params->pscch.rx_ok);

      LOG_I(NR_PHY,"%s[UE%d] %d:%d PSSCH/SCI2 Stats: TX %d, RX ok %d, RX not ok %d\n",KGRN,
                                                      ue->Mod_id, frame_rx, nr_slot_rx,
                                                      sl_phy_params->pssch.num_pssch_sci2_tx,
                                                      sl_phy_params->pssch.rx_sci2_ok,
                                                      sl_phy_params->pssch.rx_sci2_errors);
      LOG_I(NR_PHY,"%s[UE%d] %d:%d PSSCH Stats: TX %d, RX ok %d, RX not ok (%d/%d/%d/%d)\n",KGRN,
                                                      ue->Mod_id, frame_rx, nr_slot_rx,
                                                      sl_phy_params->pssch.num_pssch_tx,
                                                      sl_phy_params->pssch.rx_ok,
                                                      sl_phy_params->pssch.rx_errors[0],
                                                      sl_phy_params->pssch.rx_errors[1],
                                                      sl_phy_params->pssch.rx_errors[2],
                                                      sl_phy_params->pssch.rx_errors[3]);
      LOG_I(NR_PHY, "%s[UE%d] %d:%d PSFCH Stats: TX %d\n", KGRN,
                                                      ue->Mod_id, frame_rx, nr_slot_rx,
                                                      sl_phy_params->psfch.num_psfch_tx
                                                      );
      LOG_I(NR_PHY,"============================================\n");
  }

  if (phy_data->sl_rx_action == SL_NR_CONFIG_TYPE_RX_PSBCH){

    const int estimateSz = fp->symbols_per_slot * fp->ofdm_symbol_size;

    //VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_SLOT_FEP_PSBCH, VCD_FUNCTION_IN);
    LOG_D(PHY," ----- PSBCH RX TTI: frame.slot %d.%d ------  \n",
                                              frame_rx%1024, nr_slot_rx);

    __attribute__ ((aligned(32))) struct complex16 dl_ch_estimates[fp->nb_antennas_rx][estimateSz];
    __attribute__ ((aligned(32))) struct complex16 dl_ch_estimates_time[fp->nb_antennas_rx][fp->ofdm_symbol_size];

    // 0 for Normal Cyclic Prefix and 1 for EXT CyclicPrefix
    const int numsym = (fp->Ncp) ? SL_NR_NUM_SYMBOLS_SSB_EXT_CP
                                 : SL_NR_NUM_SYMBOLS_SSB_NORMAL_CP;

    for (int sym=0; sym<numsym;) {
      nr_slot_fep(ue,
                  fp,
                  proc,
                  sym,
                  rxdataF,
                  link_type_sl);

      start_meas(&sl_phy_params->channel_estimation_stats);
      nr_pbch_channel_estimation(ue,
                                  fp,
                                  estimateSz,
                                  dl_ch_estimates,
                                  dl_ch_estimates_time,
                                  proc,
                                  sym,
                                  sym,
                                  0,
                                  0,
                                  rxdataF,
                                  true,
                                  sl_phy_params->sl_config.sl_sync_source.rx_slss_id);
      stop_meas(&sl_phy_params->channel_estimation_stats);

      //PSBCH present in symbols 0, 5-12 for normal cp
      sym = (sym == 0) ? 5 : sym + 1;
    }

    nr_sl_psbch_rsrp_measurements(sl_phy_params,fp, rxdataF,false);

    LOG_D(PHY," ------  Decode SL-MIB: frame.slot %d.%d ------  \n",
                                                  frame_rx%1024, nr_slot_rx);

    const int psbchSuccess = nr_ue_psbch_procedures(ue, fp, proc, estimateSz,
                                                   dl_ch_estimates, phy_data, rxdataF);

    if (ue->no_timing_correction==0 && psbchSuccess == 0) {
      LOG_D(PHY,"start adjust sync slot = %d no timing %d\n", nr_slot_rx, ue->no_timing_correction);
      nr_adjust_synch_ue(fp,
                         ue,
                         proc->gNB_id,
                         fp->ofdm_symbol_size,
                         dl_ch_estimates_time,
                         frame_rx,
                         nr_slot_rx,
                         0,
                         16384);
    }
    ue->apply_timing_offset = true;

    LOG_D(PHY, "Doing N0 measurements in %s\n", __FUNCTION__);
//    nr_ue_rrc_measurements(ue, proc, rxdataF);
    //VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_SLOT_FEP_PSBCH, VCD_FUNCTION_OUT);

  }
  else if (phy_data->sl_rx_action == SL_NR_CONFIG_TYPE_RX_PSCCH){

    fapi_nr_dl_config_dci_dl_pdu_rel15_t *rel15 = &phy_data->phy_pdcch_config.pdcch_config[0];
    LOG_D(NR_PHY,"pscch_numsym = %d\n",phy_data->nr_sl_pscch_pdu.pscch_numsym);
    LOG_D(NR_PHY,"pscch_startrb = %d\n",phy_data->nr_sl_pscch_pdu.pscch_startrb);
    LOG_D(NR_PHY,"pscch_numrbs = %d\n",phy_data->nr_sl_pscch_pdu.pscch_numrbs);
    LOG_D(NR_PHY,"pscch_dmrs_scrambling_id = %d\n",phy_data->nr_sl_pscch_pdu.pscch_dmrs_scrambling_id);

    LOG_D(NR_PHY,"pscch_num_subch= %d\n",phy_data->nr_sl_pscch_pdu.num_subch);
    LOG_D(NR_PHY,"pscch_subchannel_size = %d\n",phy_data->nr_sl_pscch_pdu.subchannel_size);
    LOG_D(NR_PHY,"pscch_l_subch = %d\n",phy_data->nr_sl_pscch_pdu.l_subch);
    LOG_D(NR_PHY,"pscch_pssch_numsym = %d\n",phy_data->nr_sl_pscch_pdu.pssch_numsym);
    LOG_D(NR_PHY,"sense_pscch = %d\n",phy_data->nr_sl_pscch_pdu.sense_pscch);

    rel15->rnti = 0;
    rel15->BWPSize = phy_data->nr_sl_pscch_pdu.num_subch * phy_data->nr_sl_pscch_pdu.subchannel_size;
    rel15->BWPStart = phy_data->nr_sl_pscch_pdu.pscch_startrb;
    rel15->SubcarrierSpacing = fp->subcarrier_spacing;
    rel15->coreset.frequency_domain_resource[0] = phy_data->nr_sl_pscch_pdu.pscch_startrb;
    rel15->coreset.frequency_domain_resource[1] = phy_data->nr_sl_pscch_pdu.pscch_numrbs;
    rel15->coreset.CoreSetType = NFAPI_NR_CSET_CONFIG_PDCCH_CONFIG;
    rel15->coreset.StartSymbolIndex = 1;
    rel15->coreset.RegBundleSize = 0;
    rel15->coreset.duration = phy_data->nr_sl_pscch_pdu.pscch_numsym;
    rel15->coreset.pdcch_dmrs_scrambling_id = phy_data->nr_sl_pscch_pdu.pscch_dmrs_scrambling_id;
    rel15->coreset.scrambling_rnti = 1010;
    rel15->coreset.tci_present_in_dci = 0;

    rel15->number_of_candidates = phy_data->nr_sl_pscch_pdu.l_subch;
    rel15->num_dci_options = 1;
    rel15->dci_length_options[0] = phy_data->nr_sl_pscch_pdu.sci_1a_length;
    // L now provides the number of PRBs used by PSCCH instead of the number of CCEs
    rel15->L[0] = phy_data->nr_sl_pscch_pdu.pscch_numrbs * phy_data->nr_sl_pscch_pdu.pscch_numsym;
    // This provides the offset of the candidate of PSCCH in RBs instead of CCEs
    rel15->CCE[0] = 0;
 
    // Hold the channel estimates in frequency domain.
    int32_t pscch_est_size = ((((fp->symbols_per_slot*(fp->ofdm_symbol_size+LTE_CE_FILTER_LENGTH))+15)/16)*16);
     __attribute__ ((aligned(16))) int32_t pscch_dl_ch_estimates[4*fp->nb_antennas_rx][pscch_est_size];
    //
    for (int sym=0; sym<rel15->coreset.duration;sym++) {
      nr_slot_fep(ue,
                  fp,
                  proc,
                  1+sym,
                  rxdataF,
                  link_type_sl);
      nr_pdcch_channel_estimation(ue,
                                  proc,
                                  1,
                                  1+sym,
                                  &rel15->coreset,
                                  fp->first_carrier_offset,
                                  rel15->BWPStart,
                                  pscch_est_size,
                                  pscch_dl_ch_estimates,
                                  rxdataF);
    }

    nr_ue_pdcch_procedures(ue, proc, 1, pscch_est_size, pscch_dl_ch_estimates, phy_data, 0, rxdataF); 
    LOG_D(NR_PHY,"returned from nr_ue_pdcch_procedures\n");
  }

  if (phy_data->sl_rx_action == SL_NR_CONFIG_TYPE_RX_PSSCH_SCI) {
    LOG_D(NR_PHY,"sci2_len = %d\n",phy_data->nr_sl_pssch_sci_pdu.sci2_len);
    LOG_D(NR_PHY,"sci2_beta_offset = %d\n",phy_data->nr_sl_pssch_sci_pdu.sci2_beta_offset);
    LOG_D(NR_PHY,"sci2_alpha_times_100= %d\n",phy_data->nr_sl_pssch_sci_pdu.sci2_alpha_times_100);
    LOG_D(NR_PHY,"pssch_targetCodeRate = %d\n",phy_data->nr_sl_pssch_sci_pdu.targetCodeRate);
    LOG_D(NR_PHY,"pssch_num_layers = %d\n",phy_data->nr_sl_pssch_sci_pdu.num_layers);
    LOG_D(NR_PHY,"dmrs_symbol_position = %d\n",phy_data->nr_sl_pssch_sci_pdu.dmrs_symbol_position);
    int num_dmrs = 0;
    for (int s = 0; s < NR_NUMBER_OF_SYMBOLS_PER_SLOT; s++)
      num_dmrs += (phy_data->nr_sl_pssch_sci_pdu.dmrs_symbol_position >> s) & 1;
    LOG_D(NR_PHY,"num_dmrs = %d\n",num_dmrs);
    LOG_D(NR_PHY,"Nid = %x\n",phy_data->nr_sl_pssch_sci_pdu.Nid);

    LOG_D(NR_PHY,"startrb = %d\n",phy_data->nr_sl_pssch_sci_pdu.startrb);
    LOG_D(NR_PHY,"pscch_numsym = %d\n",phy_data->nr_sl_pssch_sci_pdu.pscch_numsym);
    LOG_D(NR_PHY,"pscch_numrbs = %d\n",phy_data->nr_sl_pssch_sci_pdu.pscch_numrbs);
    LOG_D(NR_PHY,"num_subch= %d\n",phy_data->nr_sl_pssch_sci_pdu.num_subch);
    LOG_D(NR_PHY,"subchannel_size = %d\n",phy_data->nr_sl_pssch_sci_pdu.subchannel_size);
    LOG_D(NR_PHY,"l_subch = %d\n",phy_data->nr_sl_pssch_sci_pdu.l_subch);
    LOG_D(NR_PHY,"pssch_numsym = %d\n",phy_data->nr_sl_pssch_sci_pdu.pssch_numsym);
    LOG_D(NR_PHY,"sense_pssch = %d\n",phy_data->nr_sl_pssch_sci_pdu.sense_pssch);
    ue->slsch->harq_process->pssch_pdu = &phy_data->nr_sl_pssch_sci_pdu;
    // compute number of REs containing SCI2
    int sci2_re = get_NREsci2_2(phy_data->nr_sl_pssch_sci_pdu.sci2_alpha_times_100,
                                phy_data->nr_sl_pssch_sci_pdu.sci2_len,
                                phy_data->nr_sl_pssch_sci_pdu.sci2_beta_offset,
                                phy_data->nr_sl_pssch_sci_pdu.pssch_numsym,
                                phy_data->nr_sl_pssch_sci_pdu.pscch_numsym,
                                phy_data->nr_sl_pssch_sci_pdu.pscch_numrbs,
                                phy_data->nr_sl_pssch_sci_pdu.l_subch,
                                phy_data->nr_sl_pssch_sci_pdu.subchannel_size,
                                phy_data->nr_sl_pssch_sci_pdu.targetCodeRate);
    LOG_D(NR_PHY,"Starting slot FEP for SLSCH (symbol %d to %d) pscch_numsym %d pssch_numsym %d\n",1+phy_data->nr_sl_pssch_sci_pdu.pscch_numsym,phy_data->nr_sl_pssch_sci_pdu.pssch_numsym,phy_data->nr_sl_pssch_sci_pdu.pscch_numsym,phy_data->nr_sl_pssch_sci_pdu.pssch_numsym); 
    for (int sym=1+phy_data->nr_sl_pssch_sci_pdu.pscch_numsym; sym<=phy_data->nr_sl_pssch_sci_pdu.pssch_numsym;sym++) {
      nr_slot_fep(ue,
                  fp,
                  proc,
                  sym,
                  rxdataF,
                  link_type_sl);

    }

    nr_rx_pusch(NULL,
                ue,
                proc,
                phy_data,
		rxdataF_sz,
		rxdataF,
                0,
                frame_rx,
                nr_slot_rx,
                0,
                &is_csi_rs_slot);
      NR_gNB_PUSCH *pssch_vars = &ue->pssch_vars[0];
      pssch_vars->ulsch_power_tot = 0;
      pssch_vars->ulsch_noise_power_tot = 0;
      for (int aarx = 0; aarx < fp->nb_antennas_rx; aarx++) {
        pssch_vars->ulsch_power[aarx] /= num_dmrs;
        pssch_vars->ulsch_power_tot += pssch_vars->ulsch_power[aarx];
        pssch_vars->ulsch_noise_power[aarx] /= num_dmrs;
        pssch_vars->ulsch_noise_power_tot += pssch_vars->ulsch_noise_power[aarx];
      }
      if (dB_fixed_x10(pssch_vars->ulsch_power_tot) < dB_fixed_x10(pssch_vars->ulsch_noise_power_tot) + ue->pssch_thres) {

        LOG_D(NR_PHY,
              "PSSCH not detected in %d.%d (%d,%d,%d)\n",
              frame_rx,
              nr_slot_rx,
              dB_fixed_x10(pssch_vars->ulsch_power_tot),
              dB_fixed_x10(pssch_vars->ulsch_noise_power_tot),
              ue->pssch_thres);
        pssch_vars->ulsch_power_tot = pssch_vars->ulsch_noise_power_tot;
        pssch_vars->DTX = 1;
        //if (stats)
        //  stats->ulsch_stats.DTX++;
        // nr_fill_indication(gNB, frame_rx, slot_rx, ULSCH_id, ulsch->harq_pid, 1, 1);
        //pssch_DTX++;
        //  continue;
      } else {
        LOG_D(NR_PHY,
              "PSSCH detected in %d.%d (%d,%d,%d)\n",
              frame_rx,
              nr_slot_rx,
              dB_fixed_x10(pssch_vars->ulsch_power_tot),
              dB_fixed_x10(pssch_vars->ulsch_noise_power_tot),
              ue->pssch_thres);

        pssch_vars->DTX = 0;
        int totalDecode = nr_slsch_procedures(ue, frame_rx, nr_slot_rx, 0, proc, phy_data, is_csi_rs_slot);
      }
  }

  LOG_D(PHY,"****** end Sidelink RX-Chain for AbsSubframe %d.%d ******\n",
                                                                frame_rx, nr_slot_rx);

  //VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_UE_TX_SL, VCD_FUNCTION_OUT);
  stop_meas(&sl_phy_params->phy_proc_sl_tx);

  return;
}
int phy_procedures_nrUE_SL_TX(PHY_VARS_NR_UE *ue,
                            UE_nr_rxtx_proc_t *proc,
                            nr_phy_data_tx_t *phy_data)
{

  int slot_tx = proc->nr_slot_tx;
  int frame_tx = proc->frame_tx;
  int tx_action = 0;

  const char *sl_tx_actions[] = {"PSBCH", "PSCCH_PSSCH", "PSCCH_PSSCH_PSFCH", "PSCCH_PSSCH_CSI_RS", "PSCCH_PSSCH_PSFCH_CSI_RS"};
  if (phy_data->sl_tx_action == SL_NR_CONFIG_TYPE_TX_PSCCH_PSSCH_CSI_RS || phy_data->sl_tx_action == SL_NR_CONFIG_TYPE_TX_PSCCH_PSSCH_PSFCH_CSI_RS) {
    LOG_D(NR_PHY, "Generating %s (%d.%d)\n", sl_tx_actions[phy_data->sl_tx_action - SL_NR_CONFIG_TYPE_TX_PSBCH], frame_tx, slot_tx);
  }
  sl_nr_ue_phy_params_t *sl_phy_params = &ue->SL_UE_PHY_PARAMS;
  NR_DL_FRAME_PARMS *fp = &sl_phy_params->sl_frame_params;

  //VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_UE_TX_SL,VCD_FUNCTION_IN);

  const int samplesF_per_slot = NR_SYMBOLS_PER_SLOT * fp->ofdm_symbol_size;
  c16_t txdataF_buf[fp->nb_antennas_tx * samplesF_per_slot] __attribute__((aligned(32)));
  memset(txdataF_buf, 0, sizeof(txdataF_buf));
  c16_t *txdataF[fp->nb_antennas_tx]; /* workaround to be compatible with current txdataF usage in all tx procedures. */
  for(int i=0; i< fp->nb_antennas_tx; ++i)
    txdataF[i] = &txdataF_buf[i * samplesF_per_slot];

  LOG_D(PHY,"****** start Sidelink TX-Chain for AbsSubframe %d.%d ******\n",
                                                                frame_tx, slot_tx);

  start_meas(&sl_phy_params->phy_proc_sl_tx);

  if (phy_data->sl_tx_action == SL_NR_CONFIG_TYPE_TX_PSBCH) {
    sl_nr_tx_config_psbch_pdu_t *psbch_vars = &phy_data->psbch_vars;
    nr_tx_psbch(ue, frame_tx, slot_tx, psbch_vars, txdataF);
    sl_phy_params->psbch.num_psbch_tx ++;

    tx_action = 1;
  }
  else if (phy_data->sl_tx_action == SL_NR_CONFIG_TYPE_TX_PSCCH_PSSCH ||
           phy_data->sl_tx_action == SL_NR_CONFIG_TYPE_TX_PSCCH_PSSCH_CSI_RS ||
           phy_data->sl_tx_action == SL_NR_CONFIG_TYPE_TX_PSCCH_PSSCH_PSFCH ||
           phy_data->sl_tx_action == SL_NR_CONFIG_TYPE_TX_PSCCH_PSSCH_PSFCH_CSI_RS) {
    if (phy_data->sl_tx_action >= SL_NR_CONFIG_TYPE_TX_PSCCH_PSSCH && phy_data->sl_tx_action <= SL_NR_CONFIG_TYPE_TX_PSCCH_PSSCH_PSFCH_CSI_RS)
      LOG_D(NR_PHY, "Generating %s (%d.%d)\n", sl_tx_actions[phy_data->sl_tx_action - SL_NR_CONFIG_TYPE_TX_PSBCH], frame_tx, slot_tx);
    phy_data->pscch_Nid = nr_generate_sci1(ue, txdataF[0], fp, AMP, slot_tx, &phy_data->nr_sl_pssch_pscch_pdu) &0xFFFF;
    nfapi_nr_dl_tti_csi_rs_pdu_rel15_t *csi_params = (nfapi_nr_dl_tti_csi_rs_pdu_rel15_t *)&phy_data->nr_sl_pssch_pscch_pdu.nr_sl_csi_rs_pdu;
    csi_params->scramb_id = phy_data->pscch_Nid % (1 << 10);
    nr_ue_ulsch_procedures(ue,0,frame_tx,slot_tx,0,phy_data,txdataF);

    sl_phy_params->pscch.num_pscch_tx ++;
    sl_phy_params->pssch.num_pssch_sci2_tx ++;
    sl_phy_params->pssch.num_pssch_tx ++;
    if (phy_data->sl_tx_action == SL_NR_CONFIG_TYPE_TX_PSCCH_PSSCH_CSI_RS ||
        phy_data->sl_tx_action == SL_NR_CONFIG_TYPE_TX_PSCCH_PSSCH_PSFCH_CSI_RS) {
      uint16_t beta_csirs = get_softmodem_params()->sl_mode ? (uint16_t)(AMP * (ceil(sqrt(phy_data->nr_sl_pssch_pscch_pdu.num_layers / fp->nb_antennas_tx)))) & 0xFFFF : AMP;
      LOG_D(NR_PHY, "Tx beta_csirs: %d, scramb_id %i (%d.%d)\n", beta_csirs, csi_params->scramb_id, frame_tx, slot_tx);
      nr_generate_csi_rs(fp,
                         (int32_t **)txdataF,
                         beta_csirs,
                         ue->nr_csi_info,
                         csi_params,
                         slot_tx,
                         NULL,
                         NULL,
                         NULL,
                         NULL,
                         NULL,
                         NULL,
                         NULL,
                         NULL);
    }
    if (phy_data->sl_tx_action == SL_NR_CONFIG_TYPE_TX_PSCCH_PSSCH_PSFCH ||
        phy_data->sl_tx_action == SL_NR_CONFIG_TYPE_TX_PSCCH_PSSCH_PSFCH_CSI_RS) {
      nr_generate_psfch0(ue,
                        txdataF,
                        fp,
                        AMP,
                        slot_tx,
                        &phy_data->nr_sl_pssch_pscch_pdu.psfch_pdu);
      sl_phy_params->psfch.num_psfch_tx ++;
    }
    tx_action = 1;
  }
  if (tx_action) {
    LOG_D(PHY, "Sending SL data \n");
    nr_ue_pusch_common_procedures(ue,
                                  proc->nr_slot_tx,
                                  fp,
                                  fp->nb_antennas_tx,
                                  txdataF, link_type_sl);

  }
  LOG_D(PHY,"****** end Sidelink TX-Chain for AbsSubframe %d.%d ******\n",
        frame_tx, slot_tx);
  return tx_action;
}
