(in-package :comf)

(cpl:def-cram-function move-to-poi ()
	(llif::call-nav-action-ps (llif::popPoi))
)

(cpl:def-cram-function scan-object ()
	(llif::insert-knowledge-objects(get-confident-objects))
	)

(cpl:def-cram-function move-to-poi-and-scan ()
	(move-to-poi)
	(scan-object)
)
