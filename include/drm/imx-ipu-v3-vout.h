#ifndef __DRM_IPUV3_VOUT_H
#define __DRM_IPUV3_VOUT_H

struct ipu_vout_pdata {
	struct ipu_soc *ipu;
	struct ipu_dp *dp;
	struct ipuv3_channel *ipu_ch;
};

#endif /* __DRM_IPUV3_VOUT_H */
