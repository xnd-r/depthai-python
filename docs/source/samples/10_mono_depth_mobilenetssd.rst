10 - Mono & MobilenetSSD & Encoding
======================================

This example shows how to run MobileNetv2SSD on the left grayscale camera in parallel with running the disparity depth results, displaying both the depth map and the left grayscale stream, with the bounding box from the neural network overlaid.

Demo
####

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="//www.youtube.com/embed/D_hefNl134M" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>

Source code
###########

Also `available on GitHub <https://github.com/luxonis/depthai-python/blob/gen2_develop/examples/10_mono_depth_mobilenetssd.py>`__

.. literalinclude:: ../../../examples/10_mono_depth_mobilenetssd.py
   :language: python
   :linenos:

.. include::  /includes/footer-short.rst
