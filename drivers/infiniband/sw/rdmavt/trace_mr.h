/*
 * Copyright(c) 2016 Intel Corporation.
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * BSD LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#if !defined(__RVT_TRACE_MR_H) || defined(TRACE_HEADER_MULTI_READ)
#define __RVT_TRACE_MR_H

#include <linux/tracepoint.h>
#include <linux/trace_seq.h>

#include <rdma/ib_verbs.h>
#include <rdma/rdma_vt.h>
#include <rdma/rdmavt_mr.h>

#undef TRACE_SYSTEM
#define TRACE_SYSTEM rvt_mr
DECLARE_EVENT_CLASS(
	rvt_mr_template,
	TP_PROTO(struct rvt_mregion *mr, u16 m, u16 n, void *v, size_t len),
	TP_ARGS(mr, m, n, v, len),
	TP_STRUCT__entry(
		RDI_DEV_ENTRY(ib_to_rvt(mr->pd->device))
		__field(void *, vaddr)
		__field(struct page *, page)
		__field(size_t, len)
		__field(u32, lkey)
		__field(u16, m)
		__field(u16, n)
	),
	TP_fast_assign(
		RDI_DEV_ASSIGN(ib_to_rvt(mr->pd->device));
		__entry->vaddr = v;
		__entry->page = virt_to_page(v);
		__entry->m = m;
		__entry->n = n;
		__entry->len = len;
	),
	TP_printk(
		"[%s] vaddr %p page %p m %u n %u len %ld",
		__get_str(dev),
		__entry->vaddr,
		__entry->page,
		__entry->m,
		__entry->n,
		__entry->len
	)
);

DEFINE_EVENT(
	rvt_mr_template, rvt_mr_page_seg,
	TP_PROTO(struct rvt_mregion *mr, u16 m, u16 n, void *v, size_t len),
	TP_ARGS(mr, m, n, v, len));

DEFINE_EVENT(
	rvt_mr_template, rvt_mr_fmr_seg,
	TP_PROTO(struct rvt_mregion *mr, u16 m, u16 n, void *v, size_t len),
	TP_ARGS(mr, m, n, v, len));

DEFINE_EVENT(
	rvt_mr_template, rvt_mr_user_seg,
	TP_PROTO(struct rvt_mregion *mr, u16 m, u16 n, void *v, size_t len),
	TP_ARGS(mr, m, n, v, len));

#endif /* __RVT_TRACE_MR_H */

#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE trace_mr
#include <trace/define_trace.h>
