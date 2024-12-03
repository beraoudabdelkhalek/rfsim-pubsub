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

/* Definitions for LTE Reference signals */
/* Author R. Knopp / EURECOM / OpenAirInterface.org */
#ifndef __NR_REFSIG_DEFS__H__
#define __NR_REFSIG_DEFS__H__

#include "PHY/defs_nr_UE.h"
#include "PHY/LTE_REFSIG/lte_refsig.h"

typedef struct port_freq_indices {
  uint8_t p;
  uint16_t k;
} port_freq_indices_t;

typedef struct csi_rs_params {
  uint8_t size;
  uint8_t j[16];
  uint8_t k_n[6];
  uint8_t kprime;
  uint8_t lprime;
  uint8_t ports;
  uint8_t koverline[16];
  uint8_t loverline[16];
  double rho;
  double alpha;
  uint8_t gs;
} csi_rs_params_t;

/*!\brief This function generates the NR Gold sequence (38-211, Sec 5.2.1) for the PBCH DMRS.
@param PHY_VARS_NR_UE* ue structure provides configuration, frame parameters and the pointers to the 32 bits sequence storage tables
 */
int nr_pbch_dmrs_rx(int dmrss,
                    unsigned int *nr_gold_pbch,
                    int32_t *output,
                    bool sidelink);

/*!\brief This function generates the NR Gold sequence (38-211, Sec 5.2.1) for the PDCCH DMRS.
@param PHY_VARS_NR_UE* ue structure provides configuration, frame parameters and the pointers to the 32 bits sequence storage tables
 */
int nr_pdcch_dmrs_rx(PHY_VARS_NR_UE *ue,
                     unsigned int Ns,
                     unsigned int *nr_gold_pdcch,
                     int32_t *output,
                     unsigned short p,
                     unsigned short nb_rb_corset);

int nr_pdsch_dmrs_rx(PHY_VARS_NR_UE *ue,
                     unsigned int Ns,
                     unsigned int *nr_gold_pdsch,
                     int32_t *output,
                     unsigned short p,
                     unsigned char lp,
                     unsigned short nb_pdsch_rb,
                     uint8_t config_type);

void nr_gold_pbch(PHY_VARS_NR_UE* ue);

void nr_gold_pdcch(NR_DL_FRAME_PARMS *fp, 
                   uint32_t ***nr_gold, uint16_t nid); 

void nr_gold_pdsch(PHY_VARS_NR_UE* ue,
                   int nscid,
                   uint32_t nid);

void nr_init_pusch_dmrs(PHY_VARS_NR_UE* ue,
                        uint16_t N_n_scid,
                        uint8_t n_scid);

void nr_init_pssch_dmrs_oneshot(NR_DL_FRAME_PARMS *fp,
                                uint16_t N_id,
                                uint32_t *pssch_dmrs,
                                int slot,
                                int symb);

void nr_init_csi_rs(const NR_DL_FRAME_PARMS *fp, uint32_t ***csi_rs, uint32_t Nid);
void init_nr_gold_prs(PHY_VARS_NR_UE* ue);

void get_csi_rs_freq_ind_sl(const NR_DL_FRAME_PARMS* frame_parms,
                            uint16_t n,
                            nfapi_nr_dl_tti_csi_rs_pdu_rel15_t* csi_params,
                            csi_rs_params_t* table_params,
                            port_freq_indices_t* port_freq_indices);

void get_csi_rs_params_from_table(const nfapi_nr_dl_tti_csi_rs_pdu_rel15_t *csi_params,
                                  csi_rs_params_t* table_params);
#endif
