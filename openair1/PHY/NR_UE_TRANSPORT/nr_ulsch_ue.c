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

/*! \file PHY/NR_UE_TRANSPORT/nr_ulsch.c
* \brief Top-level routines for transmission of the PUSCH TS 38.211 v 15.4.0
* \author Khalid Ahmed
* \date 2019
* \version 0.1
* \company Fraunhofer IIS
* \email: khalid.ahmed@iis.fraunhofer.de
* \note
* \warning
*/
#include <stdint.h>
#include "PHY/NR_REFSIG/dmrs_nr.h"
#include "PHY/NR_REFSIG/ptrs_nr.h"
#include "PHY/NR_REFSIG/refsig_defs_ue.h"
#include "PHY/NR_UE_TRANSPORT/nr_transport_ue.h"
#include "PHY/NR_UE_TRANSPORT/nr_transport_proto_ue.h"
#include "PHY/MODULATION/nr_modulation.h"
#include "PHY/MODULATION/modulation_common.h"
#include "common/utils/assertions.h"
#include "common/utils/nr/nr_common.h"
#include "common/utils/LOG/vcd_signal_dumper.h"
#include "PHY/NR_TRANSPORT/nr_transport_common_proto.h"
#include "PHY/NR_TRANSPORT/nr_sch_dmrs.h"
#include "PHY/NR_TRANSPORT/nr_dci.h"
#include "PHY/defs_nr_common.h"
#include "PHY/TOOLS/tools_defs.h"
#include "executables/nr-softmodem.h"
#include "executables/softmodem-common.h"
#include "PHY/NR_REFSIG/ul_ref_seq_nr.h"
#include <openair2/UTIL/OPT/opt.h>

//#define DEBUG_PUSCH_MAPPING
//#define DEBUG_MAC_PDU
//#define DEBUG_DFT_IDFT

//extern int32_t uplink_counter;

void nr_pusch_codeword_scrambling_uci(uint8_t *in,
                                      uint32_t size,
                                      uint32_t Nid,
                                      uint32_t n_RNTI,
                                      uint32_t* out)
{
  uint8_t reset, b_idx;
  uint32_t x1 = 0, x2 = 0, s = 0, temp_out = 0;

  reset = 1;
  x2 = (n_RNTI<<15) + Nid;

  for (int i=0; i<size; i++) {
    b_idx = i&0x1f;
    if (b_idx==0) {
      s = lte_gold_generic(&x1, &x2, reset);
      reset = 0;
      if (i)
        out++;
    }
    if (in[i]==NR_PUSCH_x)
      *out ^= 1<<b_idx;
    else if (in[i]==NR_PUSCH_y){
      if (b_idx!=0)
        *out ^= (*out & (1<<(b_idx-1)))<<1;
      else{

        temp_out = *(out-1);
        *out ^= temp_out>>31;

      }
    }
    else
      *out ^= (((in[i])&1) ^ ((s>>b_idx)&1))<<b_idx;
    //printf("i %d b_idx %d in %d s 0x%08x out 0x%08x\n", i, b_idx, in[i], s, *out);
  }
}

void nr_pusch_codeword_scrambling_sci(uint32_t *in,
                                      uint32_t size,
                                      uint32_t Nid,
                                      uint32_t* out)
{
  uint8_t reset, b_idx;
  uint32_t x1 = 0, x2 = 0, s = 0;

  reset = 1;
  x2 = (Nid<<15) + 1010;

  for (int i=0; i<size; i++) {
    b_idx = i&0x1f;
    if (b_idx==0) {
      s = lte_gold_generic(&x1, &x2, reset);
      reset = 0;
      if (i)
        out++;
    }
    *out ^= (((in[i])&1) ^ ((s>>b_idx)&1))<<b_idx;
    //printf("i %d b_idx %d in %d s 0x%08x out 0x%08x\n", i, b_idx, in[i], s, *out);
  }
}
void nr_pusch_codeword_scrambling_sci_2layer(uint32_t *in,
                                             uint32_t size,
                                             uint32_t Nid,
                                             uint32_t* out)
{
  uint8_t reset, b_idx;
  uint32_t x1 = 0, x2 = 0, s = 0;

  reset = 1;
  x2 = (Nid<<15) + 1010;

  for (int i=0; i<size; i+=4) {
    b_idx = i&0x1f;
    if (b_idx==0) {
      s = lte_gold_generic(&x1, &x2, reset);
      reset = 0;
      if (i)
        out++;
    }
    *out ^= (((in[i])&1) ^ ((s>>b_idx)&1))<<b_idx;
    *out ^= (((in[i+1])&1) ^ ((s>>(b_idx+1))&1))<<(b_idx+1);
    *out ^= (((in[i])&1) ^ ((s>>b_idx)&1))<<(b_idx+2);
    *out ^= (((in[i+1])&1) ^ ((s>>(b_idx+1))&1))<<(b_idx+3);
    //printf("i %d b_idx %d in %d s 0x%08x out 0x%08x\n", i, b_idx, in[i], s, *out);
  }
}
void nr_pusch_codeword_scrambling(uint8_t *in,
                                  uint32_t size,
                                  uint32_t Nid,
                                  uint32_t n_RNTI,
                                  bool uci_on_pusch,
                                  uint32_t* out)
{
  if (uci_on_pusch)
    nr_pusch_codeword_scrambling_uci(in, size, Nid, n_RNTI, out);
  else
    nr_codeword_scrambling(in, size, 0, Nid, n_RNTI, out);
}

int dmrs_pscch_mask[2] = {7,15} ;

void nr_ue_ulsch_procedures(PHY_VARS_NR_UE *UE,
                            const unsigned char harq_pid,
                            const uint32_t frame,
                            const uint8_t slot,
                            const int gNB_id,
                            nr_phy_data_tx_t *phy_data,
                            c16_t **txdataF)
{
  LOG_D(PHY,"nr_ue_ulsch_procedures hard_id %d %d.%d\n",harq_pid,frame,slot);

  int8_t Wf[2], Wt[2];
  int l_prime[2], delta;
  uint8_t nb_dmrs_re_per_rb;
  int i;
  int sample_offsetF, N_RE_prime;

  bool is_csi_rs_slot = false;
  if (phy_data->sl_tx_action == SL_NR_CONFIG_TYPE_TX_PSCCH_PSSCH_CSI_RS ||
      phy_data->sl_tx_action == SL_NR_CONFIG_TYPE_TX_PSCCH_PSSCH_PSFCH_CSI_RS)
    is_csi_rs_slot = true;
  nfapi_nr_dl_tti_csi_rs_pdu_rel15_t *csi_params = (nfapi_nr_dl_tti_csi_rs_pdu_rel15_t *)&phy_data->nr_sl_pssch_pscch_pdu.nr_sl_csi_rs_pdu;
  LOG_D(NR_PHY, "Tx start_rb %i, cdm_type %i, csi_type %i, freq_density %i, nr_of_rbs %i, row %i\n", csi_params->start_rb, csi_params->cdm_type, csi_params->csi_type, csi_params->freq_density, csi_params->nr_of_rbs, csi_params->row);
  int      N_PRB_oh = 0; // higher layer (RRC) parameter xOverhead in PUSCH-ServingCellConfig
  uint16_t number_dmrs_symbols = 0;

  NR_UE_ULSCH_t *ulsch_ue = &phy_data->ulsch;
  sl_nr_tx_config_pscch_pssch_pdu_t *pscch_pssch_pdu = &phy_data->nr_sl_pssch_pscch_pdu;
  NR_UL_UE_HARQ_t *harq_process_ul_ue = &UE->ul_harq_processes[harq_pid];
  const nfapi_nr_ue_pusch_pdu_t *pusch_pdu = &ulsch_ue->pusch_pdu;

  NR_DL_FRAME_PARMS *frame_parms = pscch_pssch_pdu == NULL ? &UE->frame_parms : &UE->SL_UE_PHY_PARAMS.sl_frame_params;
  int start_symbol          = pscch_pssch_pdu == NULL ? pusch_pdu->start_symbol_index : (1+pscch_pssch_pdu->pssch_startsym);
  uint16_t ul_dmrs_symb_pos = pscch_pssch_pdu == NULL ? pusch_pdu->ul_dmrs_symb_pos : pscch_pssch_pdu->dmrs_symbol_position;
  uint8_t number_of_symbols = pscch_pssch_pdu == NULL ? pusch_pdu->nr_of_symbols : pscch_pssch_pdu->pssch_numsym;
  uint8_t dmrs_type         = pscch_pssch_pdu == NULL ? pusch_pdu->dmrs_config_type : pusch_dmrs_type1;
  uint16_t start_rb         = pscch_pssch_pdu == NULL ? pusch_pdu->rb_start : pscch_pssch_pdu->startrb;
  uint16_t nb_rb            = pscch_pssch_pdu == NULL ? pusch_pdu->rb_size : pscch_pssch_pdu->l_subch * pscch_pssch_pdu->subchannel_size;
  uint8_t Nl                = pscch_pssch_pdu == NULL ? pusch_pdu->nrOfLayers : pscch_pssch_pdu->num_layers;
  uint8_t mod_order         = pscch_pssch_pdu == NULL ? pusch_pdu->qam_mod_order : pscch_pssch_pdu->mod_order;
  uint16_t rnti             = pscch_pssch_pdu == NULL ? pusch_pdu->rnti : 0;
  uint8_t cdm_grps_no_data  = pscch_pssch_pdu == NULL ? pusch_pdu->num_dmrs_cdm_grps_no_data : 1;
  uint16_t start_sc         = frame_parms->first_carrier_offset + (start_rb+(pscch_pssch_pdu == NULL ? pusch_pdu->bwp_start:0))*NR_NB_SC_PER_RB;
  uint16_t Tpmi             = pscch_pssch_pdu == NULL ? pusch_pdu->Tpmi : 0;
 
  if (start_sc >= frame_parms->ofdm_symbol_size)
    start_sc -= frame_parms->ofdm_symbol_size;
  ulsch_ue->Nid_cell = frame_parms->Nid_cell;
  uint8_t first_dmrs_symbol = 0;
  bool is_first_dmrs_symbol = true;
  for (int i = start_symbol; i < start_symbol + number_of_symbols; i++) {
    if((ul_dmrs_symb_pos >> i) & 0x01) {
      number_dmrs_symbols += 1;
      if (is_first_dmrs_symbol) {
        first_dmrs_symbol = i;
        is_first_dmrs_symbol = false;
      }
      if (csi_params->symb_l0 != 0)
        AssertFatal(i != csi_params->symb_l0, "CSI-RS MUST not be sent in DMRS symbol\n");
    }
  }

  if (csi_params->symb_l0 != 0)
    AssertFatal(csi_params->symb_l0 > pscch_pssch_pdu->pscch_numsym, "CSI-RS MUST not be sent in PSCCH symbol\n");
  nb_dmrs_re_per_rb = ((dmrs_type == pusch_dmrs_type1) ? 6:4)*cdm_grps_no_data;

  //LOG_I(NR_PHY,"%s TX %x : start_rb %d nb_rb %d mod_order %d Nl %d Tpmi %d bwp_start %d start_sc %d start_symbol %d num_symbols %d cdmgrpsnodata %d num_dmrs %d dmrs_re_per_rb %d\n",pscch_pssch_pdu==NULL?"PUSCH":"PSSCH",
   //     rnti,start_rb,nb_rb,mod_order,Nl,Tpmi,pscch_pssch_pdu==NULL?pusch_pdu->bwp_start:0,start_sc,start_symbol,number_of_symbols,cdm_grps_no_data,number_dmrs_symbols,nb_dmrs_re_per_rb);
  // TbD num_of_mod_symbols is set but never used
  N_RE_prime = NR_NB_SC_PER_RB*number_of_symbols - nb_dmrs_re_per_rb*number_dmrs_symbols - N_PRB_oh;
  harq_process_ul_ue->num_of_mod_symbols = N_RE_prime*nb_rb;

  /////////////////////////ULSCH coding/////////////////////////
  ///////////

  int sci2_re = pscch_pssch_pdu ? get_NREsci2(pscch_pssch_pdu->sci2_alpha_times_100,
                                              pscch_pssch_pdu->sci2_payload_len,
                                              pscch_pssch_pdu->sci2_beta_offset,
                                              pscch_pssch_pdu->pssch_numsym,
                                              pscch_pssch_pdu->pscch_numsym,
                                              pscch_pssch_pdu->pscch_numrbs,
                                              pscch_pssch_pdu->l_subch,
                                              pscch_pssch_pdu->subchannel_size,
                                              pscch_pssch_pdu->mcs,
                                              pscch_pssch_pdu->mcs_table): 0 ;
  //if (pscch_pssch_pdu) LOG_I(NR_PHY,"dmrs_symbol_position %x, pscch_numsym %d\n",pscch_pssch_pdu->dmrs_symbol_position,pscch_pssch_pdu->pscch_numsym);
  AssertFatal(pscch_pssch_pdu->pscch_numsym==2 || pscch_pssch_pdu->pscch_numsym==3,"illegal pscch_numsym %d\n",pscch_pssch_pdu->pscch_numsym);
  int sci1_dmrs_overlap = pscch_pssch_pdu ? pscch_pssch_pdu->dmrs_symbol_position & dmrs_pscch_mask[pscch_pssch_pdu->pscch_numsym-2] : 0;
  unsigned int G = (pscch_pssch_pdu==NULL) ? nr_get_G(nb_rb, number_of_symbols,
                                                      nb_dmrs_re_per_rb, number_dmrs_symbols, mod_order, Nl):
                                             nr_get_G_SL(nb_rb, number_of_symbols,6,number_dmrs_symbols,sci1_dmrs_overlap,pscch_pssch_pdu->pscch_numsym,pscch_pssch_pdu->pscch_numrbs,sci2_re,mod_order,Nl);

  G -= (is_csi_rs_slot ? csi_params->nr_of_rbs/csi_params->freq_density*mod_order*Nl : 0);
  LOG_D(NR_PHY, "%d.%d, is_csi_rs_slot %d, G %i, number_of_symbols %i, mod_order %i, freq_density %i, nr_of_rbs %i\n", frame, slot, is_csi_rs_slot, G, number_of_symbols, mod_order, csi_params->freq_density, csi_params->nr_of_rbs);
  // Following code checks, after PSCCH symbols and DMRS symbols, whether PSSCH symbols are used by SCI2 or not,
  // If true, then CSI-RS MUST not be sent in those PSSCH symbols containing SCI2.
  if (csi_params->symb_l0 != 0) {
    int32_t next_symbs_sci2_re = 0;
    int32_t sci1_re = 12 * pscch_pssch_pdu->pscch_numrbs;
    int32_t non_sci1_re = 12 * nb_rb - sci1_re;
    next_symbs_sci2_re = first_dmrs_symbol <= pscch_pssch_pdu->pscch_numsym ? sci2_re - (non_sci1_re / 2 - (non_sci1_re * (pscch_pssch_pdu->pscch_numsym - 1)) - (12 * nb_rb) / 2) : sci2_re - (12 * nb_rb) / 2;
    int8_t remaining_sci2_symb = next_symbs_sci2_re > 0 ? ceil(next_symbs_sci2_re / (12 * nb_rb)) : 0;
    int8_t non_csi_rs_symbs = pscch_pssch_pdu->pscch_numsym + 1 + remaining_sci2_symb; // 1 is for first dmrs symbol
    AssertFatal(csi_params->symb_l0 > non_csi_rs_symbs, "CSI-RS MUST not be sent in PSSCH symbol containing SCI2");
  }

  uint32_t Gsci2 = sci2_re*2*Nl;
  trace_NRpdu(DIRECTION_UPLINK,
              harq_process_ul_ue->a,
              pscch_pssch_pdu==NULL?pusch_pdu->pusch_data.tb_size:pscch_pssch_pdu->tb_size,
              WS_C_RNTI, rnti, frame, slot, 0, 0);

  if (nr_ulsch_encoding(UE, ulsch_ue, pscch_pssch_pdu,frame_parms, harq_pid, G) == -1)
    return;
 
  
  uint32_t sci2_encoded_output[sci2_re*2];
  
  if (pscch_pssch_pdu) {
    LOG_D(NR_PHY,"Generating SCI2/PSSCH with %d RE, payload %llx\n",sci2_re,*(unsigned long long*)pscch_pssch_pdu->sci2_payload);
    // do SCI2 encoding
    polar_encoder_fast((uint64_t*)pscch_pssch_pdu->sci2_payload, (void*)sci2_encoded_output, NULL,0, 1, 
                       NR_POLAR_SCI2_MESSAGE_TYPE, 
                       pscch_pssch_pdu->sci2_payload_len, sci2_re);
  }

  ///////////
  ////////////////////////////////////////////////////////////////////

  /////////////////////////ULSCH scrambling/////////////////////////
  ///////////

  uint32_t available_bits = G;
  uint32_t scrambled_output[(available_bits>>5)+1];
  uint32_t scrambled_output_sci[(Gsci2>>5)+1];
  memset(scrambled_output, 0, ((available_bits>>5)+1)*sizeof(uint32_t));
  memset(scrambled_output_sci, 0, ((Gsci2>>5)+1)*sizeof(uint32_t));

//  for (int i=0;i<(Gsci2>>5)+1;i++) LOG_I(NR_PHY,"sci2_encoded[%d] %x\n",i,sci2_encoded_output[i]); 
//  for (int g=0;g<G;g++) LOG_I(NR_PHY,"coded_output_f[%d] %d\n",g,harq_process_ul_ue->f[g]);
//  LOG_I(NR_PHY,"Scrambling with Nid %x\n",phy_data->pscch_Nid);
  nr_pusch_codeword_scrambling(harq_process_ul_ue->f,
                               G,
                               pscch_pssch_pdu==NULL ? ulsch_ue->Nid_cell : phy_data->pscch_Nid,
                               pscch_pssch_pdu==NULL ? rnti : 1010,
                               false,
                               scrambled_output);
  if (pscch_pssch_pdu && Nl==1) 
    nr_pdcch_scrambling(sci2_encoded_output,
                        Gsci2,
                        phy_data->pscch_Nid,1010,
                        scrambled_output_sci,1);
  else if (pscch_pssch_pdu)
    nr_pusch_codeword_scrambling_sci_2layer(sci2_encoded_output,
                                 Gsci2,
                                 phy_data->pscch_Nid,
                                 scrambled_output_sci);                                          
  /////////////
  //////////////////////////////////////////////////////////////////////////

  /////////////////////////ULSCH modulation/////////////////////////
  ///////////

  int max_num_re = Nl*number_of_symbols*nb_rb*NR_NB_SC_PER_RB;
  int32_t d_mod[max_num_re] __attribute__ ((aligned(16)));

  if (Gsci2 > 0) {
    nr_modulation(scrambled_output_sci, // assume one codeword for the moment
                  Gsci2,
                  2,
                  (int16_t *)d_mod);
    //for (int i=0;i<Gsci2;i+=2) LOG_I(NR_PHY,"SCI2 RE %d/%d: (%d,%d)\n",i/2,Gsci2/2,((int16_t*)d_mod)[i],((int16_t*)d_mod)[i+1]);
    int32_t d_mod2[max_num_re] __attribute__ ((aligned(16)));
    nr_modulation(scrambled_output, // assume one codeword for the moment
                  available_bits,
                  mod_order,
                  (int16_t *)d_mod2);
    LOG_D(NR_PHY,"SCI bits %d (sci2_re %d), PSSCH bits %d (PSCCH RE %d), max_re %d\n",Gsci2,sci2_re,available_bits,available_bits/mod_order,max_num_re);
    memcpy(d_mod+sci2_re,d_mod2,available_bits*sizeof(int32_t)/mod_order);
  }
  else
    nr_modulation(scrambled_output, // assume one codeword for the moment
                  available_bits,
                  mod_order,
                  (int16_t *)d_mod);
   
  ///////////
  ////////////////////////////////////////////////////////////////////////

  /////////////////////////DMRS Modulation/////////////////////////
  ///////////

  if(!pscch_pssch_pdu && pusch_pdu->ul_dmrs_scrambling_id != UE->scramblingID_ulsch[pusch_pdu->scid])  {
    UE->scramblingID_ulsch[pusch_pdu->scid] = pusch_pdu->ul_dmrs_scrambling_id;
    nr_init_pusch_dmrs(UE, pusch_pdu->scid, pusch_pdu->ul_dmrs_scrambling_id);
  } 

  uint32_t ***pusch_dmrs = UE->nr_gold_pusch_dmrs[slot];
  uint16_t n_dmrs = ((pscch_pssch_pdu==NULL ? pusch_pdu->bwp_start : 0) + start_rb + nb_rb)*((dmrs_type == pusch_dmrs_type1) ? 6:4);
  int16_t mod_dmrs[n_dmrs<<1] __attribute((aligned(16)));

  ///////////
  ////////////////////////////////////////////////////////////////////////


  /////////////////////////PTRS parameters' initialization/////////////////////////
  ///////////

  int16_t mod_ptrs[nb_rb] __attribute((aligned(16))); // assume maximum number of PTRS per pusch allocation
  uint8_t L_ptrs, K_ptrs = 0;
  uint16_t beta_ptrs = 1; // temp value until power control is implemented

  if (!pscch_pssch_pdu && (pusch_pdu->pdu_bit_map & PUSCH_PDU_BITMAP_PUSCH_PTRS)) {

    K_ptrs = pusch_pdu->pusch_ptrs.ptrs_freq_density;
    L_ptrs = 1<<pusch_pdu->pusch_ptrs.ptrs_time_density;

    beta_ptrs = 1; // temp value until power control is implemented

    ulsch_ue->ptrs_symbols = 0;

    set_ptrs_symb_idx(&ulsch_ue->ptrs_symbols,
                      number_of_symbols,
                      start_symbol,
                      L_ptrs,
                      ul_dmrs_symb_pos);
  }

  ///////////
  ////////////////////////////////////////////////////////////////////////////////

  /////////////////////////ULSCH layer mapping/////////////////////////
  ///////////

  int16_t **tx_layers = (int16_t **)malloc16_clear(Nl*sizeof(int16_t *));
  for (int nl=0; nl<Nl; nl++)
    tx_layers[nl] = (int16_t *)malloc16_clear((((available_bits<<1)/mod_order)+(sci2_re<<1))*sizeof(int32_t));

  nr_ue_layer_mapping((int16_t *)d_mod,
                      Nl,
                      (available_bits/mod_order)+sci2_re,
                      tx_layers);

  ///////////
  ////////////////////////////////////////////////////////////////////////


  //////////////////////// ULSCH transform precoding ////////////////////////
  ///////////

  l_prime[0] = 0; // single symbol ap 0

  uint16_t index;
  uint8_t u = 0, v = 0;
  int16_t *dmrs_seq = NULL;

  /// Transform-coded "y"-sequences (for definition see 38-211 V15.3.0 2018-09, subsection 6.3.1.4)
  int32_t y[max_num_re] __attribute__ ((aligned(16)));

  if (!pscch_pssch_pdu && (pusch_pdu->transform_precoding == transformPrecoder_enabled)) {

    uint32_t nb_re_pusch=nb_rb * NR_NB_SC_PER_RB;
    uint32_t y_offset = 0;
    uint16_t num_dmrs_res_per_symbol = nb_rb*(NR_NB_SC_PER_RB/2);

    // Calculate index to dmrs seq array based on number of DMRS Subcarriers on this symbol
    index = get_index_for_dmrs_lowpapr_seq(num_dmrs_res_per_symbol);
    u = pusch_pdu->dfts_ofdm.low_papr_group_number;
    v = pusch_pdu->dfts_ofdm.low_papr_sequence_number;
    dmrs_seq = dmrs_lowpaprtype1_ul_ref_sig[u][v][index];

    AssertFatal(index >= 0, "Num RBs not configured according to 3GPP 38.211 section 6.3.1.4. For PUSCH with transform precoding, num RBs cannot be multiple of any other primenumber other than 2,3,5\n");
    AssertFatal(dmrs_seq != NULL, "DMRS low PAPR seq not found, check if DMRS sequences are generated");

    LOG_D(PHY,"Transform Precoding params. u: %d, v: %d, index for dmrsseq: %d\n", u, v, index);

    for (int l = start_symbol; l < start_symbol + number_of_symbols; l++) {

      if((ul_dmrs_symb_pos >> l) & 0x01)
        /* In the symbol with DMRS no data would be transmitted CDM groups is 2*/
        continue;

      nr_dft(&y[y_offset], &((int32_t*)tx_layers[0])[y_offset], nb_re_pusch);

      y_offset = y_offset + nb_re_pusch;

      LOG_D(PHY,"Transform precoding being done on data- symbol: %d, nb_re_pusch: %d, y_offset: %d\n", l, nb_re_pusch, y_offset);

#ifdef DEBUG_PUSCH_MAPPING
      LOG_I(NR_PHY,"NR_ULSCH_UE: y_offset %u\t nb_re_pusch %u \t Symbol %d \t nb_rb %d \n",
             y_offset, nb_re_pusch, l, nb_rb);
#endif
    }

#ifdef DEBUG_DFT_IDFT
    int32_t debug_symbols[MAX_NUM_NR_RE] __attribute__ ((aligned(16)));
    int offset = 0;
    printf("NR_ULSCH_UE: available_bits: %u, mod_order: %d", available_bits,mod_order);

    for (int ll = 0; ll < (available_bits/mod_order); ll++) {
        debug_symbols[ll] = ulsch_ue->y[ll];     
    }
      
    printf("NR_ULSCH_UE: numSym: %d, num_dmrs_sym: %d", number_of_symbols,number_dmrs_symbols);
    for (int ll = 0; ll < (number_of_symbols-number_dmrs_symbols); ll++) {
      nr_idft(&debug_symbols[offset], nb_re_pusch);
      offset = offset + nb_re_pusch;
    }
    LOG_M("preDFT_all_symbols.m","UE_preDFT", tx_layers[0],number_of_symbols*nb_re_pusch,1,1);
    LOG_M("postDFT_all_symbols.m","UE_postDFT", y,number_of_symbols*nb_re_pusch,1,1);
    LOG_M("DEBUG_IDFT_SYMBOLS.m","UE_Debug_IDFT", debug_symbols,number_of_symbols*nb_re_pusch,1,1);
    LOG_M("UE_DMRS_SEQ.m","UE_DMRS_SEQ", dmrs_seq,nb_re_pusch,1,1);
#endif

  }
  
  ///////////
  ////////////////////////////////////////////////////////////////////////



  /////////////////////////ULSCH RE mapping/////////////////////////
  ///////////

  int encoded_length = frame_parms->N_RB_UL*14*NR_NB_SC_PER_RB*mod_order*Nl;
  int16_t **tx_precoding = (int16_t **)malloc16_clear(Nl*sizeof(int16_t *));
  for (int nl=0; nl<Nl; nl++)
    tx_precoding[nl] = (int16_t *)malloc16_clear((encoded_length<<1)*sizeof(int16_t));

  for (int nl=0; nl < Nl; nl++) {
    uint8_t k_prime = 0;
    uint16_t m = 0;
    
#ifdef DEBUG_PUSCH_MAPPING
    LOG_I(NR_PHY,"NR_ULSCH_UE: Value of CELL ID %d /t, u %d \n", frame_parms->Nid_cell, u);
#endif

    int dmrs_port = get_dmrs_port(nl,pscch_pssch_pdu ? nl : pusch_pdu->dmrs_ports);
    if (dmrs_port < 0) return;
    // DMRS params for this dmrs port
    get_Wt(Wt, dmrs_port, dmrs_type);
    get_Wf(Wf, dmrs_port, dmrs_type);
    delta = get_delta(dmrs_port, dmrs_type);

    for (int l=start_symbol; l<start_symbol+number_of_symbols; l++) {

      uint16_t k = start_sc;
      uint16_t n = 0;
      uint8_t is_dmrs_sym = 0;
      uint8_t is_csi_rs_sym = 0;
      uint8_t is_ptrs_sym = 0;
      uint16_t dmrs_idx = 0, ptrs_idx = 0;
      int16_t csi_rs_rb = csi_params->start_rb;
      int is_pscch_sym = 0;
      if (pscch_pssch_pdu && l<(start_symbol + pscch_pssch_pdu->pscch_numsym)) { 
        is_pscch_sym = 1; 
      }

      if (is_csi_rs_slot && l == csi_params->symb_l0) {
        is_csi_rs_sym = 1;
      }

      if ((ul_dmrs_symb_pos >> l) & 0x01) {
        is_dmrs_sym = 1;

        if (pscch_pssch_pdu || (pusch_pdu->transform_precoding == transformPrecoder_disabled)){
        
          if (dmrs_type == pusch_dmrs_type1)
            dmrs_idx = ((pscch_pssch_pdu==NULL ? pusch_pdu->bwp_start : 0) + start_rb)*6;
          else
            dmrs_idx = ((pscch_pssch_pdu==NULL ? pusch_pdu->bwp_start : 0) + start_rb)*4;

          // TODO: performance improvement, we can skip the modulation of DMRS symbols outside the bandwidth part
          // Perform this on gold sequence, not required when SC FDMA operation is done,
          LOG_D(PHY,"DMRS in symbol %d\n",l);
          if (pscch_pssch_pdu == NULL) 
            nr_modulation(pusch_dmrs[l][pusch_pdu->scid], n_dmrs*2, DMRS_MOD_ORDER, mod_dmrs); // currently only codeword 0 is modulated. Qm = 2 as DMRS is QPSK modulated
          else {
             uint32_t pssch_dmrs[((frame_parms->N_RB_UL * 12) >> 5) + 1];
             nr_init_pssch_dmrs_oneshot(frame_parms,phy_data->pscch_Nid,pssch_dmrs,slot,l);
             nr_modulation(pssch_dmrs, n_dmrs*2, DMRS_MOD_ORDER, mod_dmrs); // currently only codeword 0 is modulated. Qm = 2 as DMRS is QPSK modulated
          }
        } else {
          dmrs_idx = 0;
        }
      } else if (pusch_pdu->pdu_bit_map & PUSCH_PDU_BITMAP_PUSCH_PTRS) {

        AssertFatal(pusch_pdu->transform_precoding == transformPrecoder_disabled, "PTRS NOT SUPPORTED IF TRANSFORM PRECODING IS ENABLED\n");

        if(is_ptrs_symbol(l, ulsch_ue->ptrs_symbols)) {
          is_ptrs_sym = 1;
          nr_modulation(pusch_dmrs[l][pusch_pdu->scid], nb_rb, DMRS_MOD_ORDER, mod_ptrs);
        }
      }

      for (i=0; i< nb_rb*NR_NB_SC_PER_RB; i++) {
        uint8_t is_dmrs = 0;
        uint8_t is_ptrs = 0;
        uint8_t is_csi_rs = 0;

        if (is_pscch_sym && i==(pscch_pssch_pdu->startrb)) {
           i+=(pscch_pssch_pdu->pscch_numrbs*NR_NB_SC_PER_RB);
           k+=(pscch_pssch_pdu->pscch_numrbs*NR_NB_SC_PER_RB);
           if (is_dmrs_sym) { 
              dmrs_idx+=(6*pscch_pssch_pdu->pscch_numrbs);
              n+=(3*pscch_pssch_pdu->pscch_numrbs);
           }
        }

        LOG_D(NR_PHY, "symbol %d re %d/%d k %d\n", l, i, nb_rb*NR_NB_SC_PER_RB, k);
        sample_offsetF = l*frame_parms->ofdm_symbol_size + k;

        if (is_dmrs_sym) {
          if (k == ((start_sc+get_dmrs_freq_idx_ul(n, k_prime, delta, dmrs_type))%frame_parms->ofdm_symbol_size))
            is_dmrs = 1;
        } else if (is_ptrs_sym) {
          is_ptrs = is_ptrs_subcarrier(k,
                                       rnti,
                                       nl,
                                       dmrs_type,
                                       K_ptrs,
                                       nb_rb,
                                       pusch_pdu->pusch_ptrs.ptrs_ports_list[0].ptrs_re_offset,
                                       start_sc,
                                       frame_parms->ofdm_symbol_size);
        }

        if (is_csi_rs_sym) {
          if ((k >= csi_params->start_rb * NR_NB_SC_PER_RB) && (i % NR_NB_SC_PER_RB == 0) && (csi_rs_rb < csi_params->nr_of_rbs)) {
            csi_rs_params_t table_params;
            get_csi_rs_params_from_table(csi_params, &table_params);
            port_freq_indices_t *port_freq_indices = (port_freq_indices_t *)malloc(table_params.ports*sizeof(port_freq_indices));
            get_csi_rs_freq_ind_sl(frame_parms, csi_rs_rb, csi_params, &table_params, port_freq_indices);
            if (k == port_freq_indices[nl].k) {
              is_csi_rs = 1;
              csi_rs_rb++;
              LOG_D(NR_PHY, "Tx port_freq_indices.p %i, port_freq_indices.k %d, is_csi_rs %d, k = %i, RE %i, csi_rs_rb %i\n", port_freq_indices[nl].p, port_freq_indices[nl].k, is_csi_rs, k, i, csi_rs_rb);
            }
            free(port_freq_indices);
            port_freq_indices = NULL;
          }
        }

        if (is_dmrs == 1) {
          // if transform precoding is enabled
          if (!pscch_pssch_pdu && pusch_pdu->transform_precoding == transformPrecoder_enabled) {
            ((int16_t*)tx_precoding[nl])[(sample_offsetF)<<1] = (Wt[l_prime[0]]*Wf[k_prime]*AMP*dmrs_seq[2*dmrs_idx]) >> 15;
            ((int16_t*)tx_precoding[nl])[((sample_offsetF)<<1) + 1] = (Wt[l_prime[0]]*Wf[k_prime]*AMP*dmrs_seq[(2*dmrs_idx) + 1]) >> 15;
          } else {
            ((int16_t*)tx_precoding[nl])[(sample_offsetF)<<1] = (Wt[l_prime[0]]*Wf[k_prime]*AMP*mod_dmrs[dmrs_idx<<1]) >> 15;
            ((int16_t*)tx_precoding[nl])[((sample_offsetF)<<1) + 1] = (Wt[l_prime[0]]*Wf[k_prime]*AMP*mod_dmrs[(dmrs_idx<<1) + 1]) >> 15;
          }

#ifdef DEBUG_PUSCH_MAPPING
          LOG_I(NR_PHY,"DMRS: Layer: %d\t, dmrs_idx %d\t l %d \t k %d \t k_prime %d \t n %d \t dmrs: %d %d\n",
                 nl, dmrs_idx, l, k, k_prime, n, ((int16_t*)tx_precoding[nl])[(sample_offsetF)<<1],
                 ((int16_t*)tx_precoding[nl])[((sample_offsetF)<<1) + 1]);
#endif

          dmrs_idx++;
          k_prime++;
          k_prime&=1;
          n+=(k_prime)?0:1;
      
        }  else if (is_ptrs == 1) {
          ((int16_t*)tx_precoding[nl])[(sample_offsetF)<<1] = (beta_ptrs*AMP*mod_ptrs[ptrs_idx<<1]) >> 15;
          ((int16_t*)tx_precoding[nl])[((sample_offsetF)<<1) + 1] = (beta_ptrs*AMP*mod_ptrs[(ptrs_idx<<1) + 1]) >> 15;
          ptrs_idx++;
        } else if (!is_dmrs_sym || allowed_xlsch_re_in_dmrs_symbol(k, start_sc, frame_parms->ofdm_symbol_size, cdm_grps_no_data, dmrs_type)) {
          if (pscch_pssch_pdu || pusch_pdu->transform_precoding == transformPrecoder_disabled) {
            if (!is_csi_rs) {
              ((int16_t*)tx_precoding[nl])[(sample_offsetF)<<1]       = ((int16_t *)tx_layers[nl])[m<<1];
              ((int16_t*)tx_precoding[nl])[((sample_offsetF)<<1) + 1] = ((int16_t *)tx_layers[nl])[(m<<1) + 1];
            } else {
              ((int16_t*)tx_precoding[nl])[(sample_offsetF)<<1]       = 0;
              ((int16_t*)tx_precoding[nl])[((sample_offsetF)<<1) + 1] = 0;
            }
          }
          else {
            ((int16_t*)tx_precoding[nl])[(sample_offsetF)<<1]       = ((int16_t *) y)[m<<1];
            ((int16_t*)tx_precoding[nl])[((sample_offsetF)<<1) + 1] = ((int16_t *) y)[(m<<1) + 1];
          }

#ifdef DEBUG_PUSCH_MAPPING
          LOG_I(NR_PHY,"DATA: layer %d\t m %d\t l %d \t k %d \t tx_precoding: %d %d\n",
                 nl, m, l, k, ((int16_t*)tx_precoding[nl])[(sample_offsetF)<<1],
                 ((int16_t*)tx_precoding[nl])[((sample_offsetF)<<1) + 1]);
#endif

          if (!is_csi_rs)
            m++;

        } else {
          ((int16_t*)tx_precoding[nl])[(sample_offsetF)<<1]       = 0;
          ((int16_t*)tx_precoding[nl])[((sample_offsetF)<<1) + 1] = 0;
        }

        if (++k >= frame_parms->ofdm_symbol_size)
          k -= frame_parms->ofdm_symbol_size;
      } //for (i=0; i< nb_rb*NR_NB_SC_PER_RB; i++) 
    }//for (l=start_symbol; l<start_symbol+number_of_symbols; l++)
  }//for (nl=0; nl < Nl; nl++)



  /////////////////////////ULSCH precoding/////////////////////////
  ///////////
  ///Layer Precoding and Antenna port mapping
  // tx_layers 0-3 are mapped on antenna ports
  // The precoding info is supported by nfapi such as num_prgs, prg_size, prgs_list and pm_idx
  // The same precoding matrix is applied on prg_size RBs, Thus
  //        pmi = prgs_list[rbidx/prg_size].pm_idx, rbidx =0,...,rbSize-1
  // The Precoding matrix:
  for (int ap=0; ap<frame_parms->nb_antennas_tx; ap++) {
    for (int l=start_symbol; l<start_symbol+number_of_symbols; l++) {
      uint16_t k = start_sc;
      int is_pscch_sym = 0;
      if (pscch_pssch_pdu && l<(start_symbol + pscch_pssch_pdu->pscch_numsym)) { 
        is_pscch_sym = 1; 
      }

      for (int rb=0; rb<nb_rb; rb++) {
        if (is_pscch_sym && rb==(pscch_pssch_pdu->startrb)) {
           k+=(pscch_pssch_pdu->pscch_numrbs*NR_NB_SC_PER_RB);
	   if (k>=frame_parms->ofdm_symbol_size) k-=frame_parms->ofdm_symbol_size;
	   rb=pscch_pssch_pdu->startrb+pscch_pssch_pdu->pscch_numrbs;
        }
        //get pmi info
        uint8_t pmi=Tpmi;
          
        if (pmi == 0) {//unitary Precoding
          if (k + NR_NB_SC_PER_RB <= frame_parms->ofdm_symbol_size) { // RB does not cross DC
            if (ap<Nl) 
              memcpy(&txdataF[ap][l*frame_parms->ofdm_symbol_size  + k],
                     &tx_precoding[ap][2*(l*frame_parms->ofdm_symbol_size + k)],
                     NR_NB_SC_PER_RB*sizeof(int32_t));
            else
              memset(&txdataF[ap][l*frame_parms->ofdm_symbol_size + k],
                     0,
                     NR_NB_SC_PER_RB*sizeof(int32_t));
          } else { // RB does cross DC
            int neg_length = frame_parms->ofdm_symbol_size - k;
            int pos_length = NR_NB_SC_PER_RB - neg_length;
            if (ap<Nl) {
              memcpy(&txdataF[ap][l*frame_parms->ofdm_symbol_size + k],
                     &tx_precoding[ap][2*(l*frame_parms->ofdm_symbol_size + k)],
                     neg_length*sizeof(int32_t));
              memcpy(&txdataF[ap][l*frame_parms->ofdm_symbol_size],
                     &tx_precoding[ap][2*(l*frame_parms->ofdm_symbol_size)],
                     pos_length*sizeof(int32_t));
            } else {
              memset(&txdataF[ap][l*frame_parms->ofdm_symbol_size + k],
                     0,
                     neg_length*sizeof(int32_t));
              memset(&txdataF[ap][l*frame_parms->ofdm_symbol_size],
                     0,
                     pos_length*sizeof(int32_t));
            }
          }
          k += NR_NB_SC_PER_RB;
          if (k >= frame_parms->ofdm_symbol_size) {
            k -= frame_parms->ofdm_symbol_size;
          }
        }
        else {
          //get the precoding matrix weights:
          const char *W_prec;
          switch (frame_parms->nb_antennas_tx) {
            case 1://1 antenna port
              W_prec = nr_W_1l_2p[pmi][ap];
              break;
            case 2://2 antenna ports
              if (Nl == 1)//1 layer
                W_prec = nr_W_1l_2p[pmi][ap];
              else//2 layers
                W_prec = nr_W_2l_2p[pmi][ap];
              break;
            case 4://4 antenna ports
              if (Nl == 1)//1 layer
                W_prec = nr_W_1l_4p[pmi][ap];
              else if (Nl == 2)//2 layers
                W_prec = nr_W_2l_4p[pmi][ap];
              else if (Nl == 3)//3 layers
                W_prec = nr_W_3l_4p[pmi][ap];
              else//4 layers
                W_prec = nr_W_4l_4p[pmi][ap];
              break;
            default:
              LOG_D(PHY,"Precoding 1,2, or 4 antenna ports are currently supported\n");
              W_prec = nr_W_1l_2p[pmi][ap];
              break;
          }

          for (int i=0; i<NR_NB_SC_PER_RB; i++) {
            int32_t re_offset = l*frame_parms->ofdm_symbol_size + k;
            int32_t precodatatx_F = nr_layer_precoder(tx_precoding, W_prec, Nl, re_offset);
            ((int16_t*)txdataF[ap])[(re_offset<<1)] = ((int16_t *) &precodatatx_F)[0];
            ((int16_t*)txdataF[ap])[(re_offset<<1) + 1] = ((int16_t *) &precodatatx_F)[1];
                            
            if (++k >= frame_parms->ofdm_symbol_size) {
              k -= frame_parms->ofdm_symbol_size;
            }
          }
        }
      } //RB loop
    } // symbol loop
  }// port loop

  NR_UL_UE_HARQ_t *harq_process_ulsch=NULL;
  harq_process_ulsch = &UE->ul_harq_processes[harq_pid];
  harq_process_ulsch->status = SCH_IDLE;

  for (int nl = 0; nl < Nl; nl++) {
    free_and_zero(tx_layers[nl]);
    free_and_zero(tx_precoding[nl]);
  }
  free_and_zero(tx_layers);
  free_and_zero(tx_precoding);

  ///////////
  ////////////////////////////////////////////////////////////////////////
}

uint8_t nr_ue_pusch_common_procedures(PHY_VARS_NR_UE *UE,
                                      const uint8_t slot,
                                      const NR_DL_FRAME_PARMS *frame_parms,
                                      const uint8_t n_antenna_ports,
                                      c16_t **txdataF,
                                      uint32_t linktype)
{
  const int tx_offset = frame_parms->get_samples_slot_timestamp(slot, frame_parms, 0);

  c16_t **txdata = UE->common_vars.txData;

  LOG_D(NR_PHY,"Applying TX rotation for slot %d linktype %d\n",slot,linktype);
  for(int ap = 0; ap < n_antenna_ports; ap++) {
    apply_nr_rotation_TX(frame_parms,
                         txdataF[ap],
                         frame_parms->symbol_rotation[linktype],
                         slot,
                         frame_parms->N_RB_UL,
                         0,
                         NR_NUMBER_OF_SYMBOLS_PER_SLOT);
  }

  for (int ap = 0; ap < n_antenna_ports; ap++) {
    if (frame_parms->Ncp == 1) { // extended cyclic prefix
      PHY_ofdm_mod((int *)txdataF[ap],
                   (int *)&txdata[ap][tx_offset],
                   frame_parms->ofdm_symbol_size,
                   12,
                   frame_parms->nb_prefix_samples,
                   CYCLIC_PREFIX);
    } else { // normal cyclic prefix
      nr_normal_prefix_mod(txdataF[ap],
                           &txdata[ap][tx_offset],
                           14,
                           frame_parms,
                           slot);
    }
  }

  ///////////
  ////////////////////////////////////////////////////
  return 0;
}

int8_t clean_UE_ulsch(PHY_VARS_NR_UE *UE, uint8_t gNB_id)
{
  for (int harq_pid = 0; harq_pid < NR_MAX_ULSCH_HARQ_PROCESSES; harq_pid++) {
    NR_UL_UE_HARQ_t *ul_harq_process = &UE->ul_harq_processes[harq_pid];
    ul_harq_process->tx_status = NEW_TRANSMISSION_HARQ;
    ul_harq_process->status = SCH_IDLE;
    ul_harq_process->round = 0;
    ul_harq_process->first_tx = 1;
  }
  return 0;
}
