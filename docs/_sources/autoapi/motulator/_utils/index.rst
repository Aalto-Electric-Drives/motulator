:orphan:

:py:mod:`motulator._utils`
==========================

.. py:module:: motulator._utils

.. autoapi-nested-parse::

   Utils.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator._utils.Bunch




.. py:class:: Bunch(**kwargs)


   Bases: :py:obj:`dict`

   
   Container object exposing keys as attributes.

   Bunch objects are sometimes used as an output for functions and methods.
   They extend dictionaries by enabling values to be accessed by key,
   `bunch["value_key"]`, or by an attribute, `bunch.value_key`.

   .. rubric:: Examples

   >>> from sklearn.utils import Bunch
   >>> b = Bunch(a=1, b=2)
   >>> b['b']
   2
   >>> b.b
   2
   >>> b.a = 3
   >>> b['a']
   3
   >>> b.c = 6
   >>> b['c']
   6















   ..
       !! processed by numpydoc !!

