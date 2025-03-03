RGB Full Resolution Saver
=========================

This example does its best to save full-resolution 3840x2160 .jpeg files as fast at it can from the
RGB sensor. It serves as an example of recording high resolution to disk for the purposes of
high-resolution ground-truth data. We also recently added the options to save isp - YUV420p
uncompressed frames, processed by ISP, and raw - BayerRG (R_Gr_Gb_B), as read from sensor,
10-bit packed.

Be careful, this example saves pictures to your host storage. So if you leave
it running, you could fill up your storage on your host.

.. rubric:: Similiar samples:

- :ref:`Mono Full Resolution Saver`

Demo
####

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="https://www.youtube.com/embed/v7XuW6vq384" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>

Setup
#####

.. include::  /includes/install_from_pypi.rst

Source code
###########

.. tabs::

    .. tab:: Python

        Also `available on GitHub <https://github.com/luxonis/depthai-python/blob/main/examples/VideoEncoder/rgb_full_resolution_saver.py>`__

        .. literalinclude:: ../../../../examples/VideoEncoder/rgb_full_resolution_saver.py
           :language: python
           :linenos:

    .. tab:: C++

        Also `available on GitHub <https://github.com/luxonis/depthai-core/blob/main/examples/VideoEncoder/rgb_full_resolution_saver.cpp>`__

        .. literalinclude:: ../../../../depthai-core/examples/VideoEncoder/rgb_full_resolution_saver.cpp
           :language: cpp
           :linenos:

.. include::  /includes/footer-short.rst
