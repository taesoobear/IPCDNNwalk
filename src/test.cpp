Get the necessary headers for numpy components and set up necessary namespaces:

#include <boost/python/numpy.hpp>
#include <iostream>

namespace p = boost::python;
namespace np = boost::python::numpy;
Initialise the Python runtime, and the numpy module. Failure to call these results in segmentation errors:

int main(int argc, char **argv)
{
  Py_Initialize();
  np::initialize();

Zero filled n-dimensional arrays can be created using the shape and data-type of the array as a parameter. Here, the shape is 3x3 and the datatype is the built-in float type:

p::tuple shape = p::make_tuple(3, 3);
np::dtype dtype = np::dtype::get_builtin<float>();
np::ndarray a = np::zeros(shape, dtype);
You can also create an empty array like this

np::ndarray b = np::empty(shape,dtype);
Print the original and reshaped array. The array a which is a list is first converted to a string, and each value in the list is extracted using extract< >:

  std::cout << "Original array:\n" << p::extract<char const *>(p::str(a)) << std::endl;

  // Reshape the array into a 1D array
  a = a.reshape(p::make_tuple(9));
  // Print it again.
  std::cout << "Reshaped array:\n" << p::extract<char const *>(p::str(a)) << std::endl;
}


UPDATE: the library described in my original answer (https://github.com/ndarray/Boost.NumPy) has been integrated directly into Boost.Python as of Boost 1.63, and hence the standalone version is now deprecated. The text below now corresponds to the new, integrated version (only the namespace has changed).

Boost.Python now includes a moderately complete wrapper of the NumPy C-API into a Boost.Python interface. It's pretty low-level, and mostly focused on how to address the more difficult problem of how to pass C++ data to and from NumPy without copying, but here's how you'd do a copied std::vector return with that:

#include "boost/python/numpy.hpp"

namespace bp = boost::python;
namespace bn = boost::python::numpy;

std::vector<double> myfunc(...);

bn::ndarray mywrapper(...) {
    std::vector<double> v = myfunc(...);
    Py_intptr_t shape[1] = { v.size() };
    bn::ndarray result = bn::zeros(1, shape, bn::dtype::get_builtin<double>());
    std::copy(v.begin(), v.end(), reinterpret_cast<double*>(result.get_data()));
    return result;
}

BOOST_PYTHON_MODULE(example) {
    bn::initialize();
    bp::def("myfunc", mywrapper);
}

May be very nice if I could actually get to the code but github seems to be blocked here, or something else is wrong because I'm getting a broken link. Surely there must be a way to populate a boost::python::numeric::array with data from a simple std::vector without having to get some 3rd party library. It would help if boost's documentation actually gave you documentation on the member functions rather than reproducing the uncommented header. – CashCow Jan 8 '13 at 13:43
I can't make an edit because it's too minor, but it should be bn::zeros, not bp::zeros. – Gabriel Jul 23 '14 at 14:29
I could not make this work (Ubuntu 14.04). What would be an example for (...)?, what is bn::initialize() supposed to do?. Also the example seems outdated -> When I try in include boost/numpy.hpp I get fatal error: boost/numpy.hpp: No such file or directory – mcExchange Jun 28 '17 at 13:23 

19

A solution that doesn't require you to download any special 3rd party C++ library (but you need numpy).

#include <numpy/ndarrayobject.h> // ensure you include this header

boost::python::object stdVecToNumpyArray( std::vector<double> const& vec )
{
      npy_intp size = vec.size();

     /* const_cast is rather horrible but we need a writable pointer
        in C++11, vec.data() will do the trick
        but you will still need to const_cast
      */

      double * data = size ? const_cast<double *>(&vec[0]) 
        : static_cast<double *>(NULL); 

    // create a PyObject * from pointer and data 
      PyObject * pyObj = PyArray_SimpleNewFromData( 1, &size, NPY_DOUBLE, data );
      boost::python::handle<> handle( pyObj );
      boost::python::numeric::array arr( handle );

    /* The problem of returning arr is twofold: firstly the user can modify
      the data which will betray the const-correctness 
      Secondly the lifetime of the data is managed by the C++ API and not the 
      lifetime of the numpy array whatsoever. But we have a simple solution..
     */

       return arr.copy(); // copy the object. numpy owns the copy now.
  }
Of course you might write a function from double * and size, which is generic then invoke that from the vector by extracting this info. You could also write a template but you'd need some kind of mapping from data type to the NPY_TYPES enum.

shareimprove this answer
edited Feb 6 '14 at 10:18
answered Jan 9 '13 at 10:16

CashCow
26.6k34879
2
Thanks for this example. Just a heads up, I had to use numeric::array::set_module_and_type("numpy", "ndarray"); or I would get the python runtime error "ImportError: No module named 'Numeric' or its type 'ArrayType' did not follow the NumPy protocol" – PiQuer Aug 29 '13 at 14:51
Thanks @PiQuer, it helped – VforVitamin Dec 14 '14 at 0:23
Why are you const_casting if you can just make the argument a non-const reference? – rubenvb Apr 17 '15 at 11:52
@rubenvb Because we want the argument to be a const reference. We are not actually going to modify the data, but we need to workaround the fact that PyArray_SimpleNewFromData requires a double* – CashCow Apr 17 '15 at 11:56 
1
Note that unlike many of my answers on StackOverflow this was a situation where I actually needed it, came here, found the question but no adequate answer. Then worked it out and came back to post it. – CashCow Apr 17 '15 at 11:59
show 7 more comments
 
10

It's a bit late, but after many unsuccessful tries I found a way to expose c++ arrays as numpy arrays directly. Here is a short C++11 example using boost::python and Eigen:

#include <numpy/ndarrayobject.h>
#include <boost/python.hpp>

#include <Eigen/Core>

// c++ type
struct my_type {
  Eigen::Vector3d position;
};


// wrap c++ array as numpy array
static boost::python::object wrap(double* data, npy_intp size) {
  using namespace boost::python;

  npy_intp shape[1] = { size }; // array size
  PyObject* obj = PyArray_New(&PyArray_Type, 1, shape, NPY_DOUBLE, // data type
                              NULL, data, // data pointer
                              0, NPY_ARRAY_CARRAY, // NPY_ARRAY_CARRAY_RO for readonly
                              NULL);
  handle<> array( obj );
  return object(array);
}



// module definition
BOOST_PYTHON_MODULE(test)
{
  // numpy requires this
  import_array();

  using namespace boost::python;

  // wrapper for my_type
  class_< my_type >("my_type")
    .add_property("position", +[](my_type& self) -> object {
        return wrap(self.position.data(), self.position.size());
      });

}
The example describes a "getter" for the property. For the "setter", the easiest way is to assign the array elements manually from a boost::python::object using a boost::python::stl_input_iterator<double>.

shareimprove this answer
answered Dec 1 '15 at 14:55

max
577617
Could you tell me how to setup my project to be able to use the numpy header? Do I need to compile some libraries? Or is it enough to include the numpy header? – NOhs Jan 26 '16 at 11:43
1
I got the numpy header directory using: python -c "import numpy; print numpy.get_include()" – max Jan 26 '16 at 16:32 
Ok. That worked, thanks. but the compiler complains that import_array() is returning a value, while init_module_... is a 'void' function. – NOhs Jan 26 '16 at 16:39
1
Ok, so it seems to be related with how the import_array() macro was change from Python 2 to Python 3 to now return something. Here is a (ugly) solution that keeps it version independent: mail.scipy.org/pipermail/numpy-discussion/2010-December/… – NOhs Jan 26 '16 at 17:06
1
Apparently, boost::python now provides direct access to numpy arrays: boost.org/doc/libs/1_63_0/libs/python/doc/html/numpy/tutorial/… can't get it to link though :-/ – max Mar 11 '17 at 23:26
show 6 more comments
 
2

Doing it using the numpy api directly is not necessarily difficult, but I use boost::multiarray regularly for my projects and find it convenient to transfer the shapes of the array between the C++/Python boundary automatically. So, here is my recipe. Use http://code.google.com/p/numpy-boost/, or better yet, this version of the numpy_boost.hpp header; which is a better fit for multi-file boost::python projects, although it uses some C++11. Then, from your boost::python code, use something like this:

PyObject* myfunc(/*....*/)
{
   // If your data is already in a boost::multiarray object:
   // numpy_boost< double, 1 > to_python( numpy_from_boost_array(result_cm) );
   // otherwise:
   numpy_boost< double, 1> to_python( boost::extents[n] );
   std::copy( my_vector.begin(), my_vector.end(), to_python.begin() );

   PyObject* result = to_python.py_ptr();
   Py_INCREF( result );

   return result;
}
shareimprove this answer
answered May 22 '12 at 13:41

dsign
7,07933467
What would be the correct way to return a py::object (py=boost::python)? I have PyObject* result=numpy_boost<double,2>(numpy_from_boost_array(...)).py_ptr(); and return py::object(py::handle<>(py::borrowed(o))); but that crashes. Hint? – eudoxos May 29 '12 at 10:53 
PS. the crash is at line 229 of the dropbox version, line a = (PyArrayObject*)PyArray_SimpleNew(NDims, shape, detail::numpy_type_map<T>::typenum);. Strange. – eudoxos May 29 '12 at 11:05
1
@eudoxos You might have a problem with the PY_ARRAY_UNIQUE_SYMBOL and NO_IMPORT_ARRAY macros, as well as import_array, as your crash is exactly when the array is created, which needs a call (I think) through certain pointer table that numpy needs (initialized with import_array() ). – dsign May 29 '12 at 13:00 
The link to the C++11 version is broken. Would you mind fixing that? – Zendel Aug 24 '17 at 0:27
add a comment

1

I looked at the available answers and thought, "this will be easy". I proceeded to spend hours attempting what seemed like a trivial examples/adaptations of the answers.

Then I implemented @max's answer exactly (had to install Eigen) and it worked fine, but I still had trouble adapting it. My problems were mostly (by number) silly, syntax mistakes, but additionally I was using a pointer to a copied std::vector's data after the vector seemed to be dropped off the stack.

In this example, a pointer to the std::vector is returned, but also you could return the size and data() pointer or use any other implementation that gives your numpy array access to the underlying data in a stable manner (i.e. guaranteed to exist):

class_<test_wrap>("test_wrap")
    .add_property("values", +[](test_wrap& self) -> object {
            return wrap(self.pvalues()->data(),self.pvalues()->size());
        })
    ;
For test_wrap with a std::vector<double> (normally pvalues() might just return the pointer without populating the vector):

class test_wrap {
public:
    std::vector<double> mValues;
    std::vector<double>* pvalues() {
        mValues.clear();
        for(double d_ = 0.0; d_ < 4; d_+=0.3)
        {
            mValues.push_back(d_);
        }
        return &mValues;
    }
};
The full example is on Github so you can skip the tedious transcription steps and worry less about build, libs, etc. You should be able to just do the following and get a functioning example (if you have the necessary features installed and your path setup already):

git clone https://github.com/ransage/boost_numpy_example.git
cd boost_numpy_example
# Install virtualenv, numpy if necessary; update path (see below*)
cd build && cmake .. && make && ./test_np.py
This should give the output:

# cmake/make output
values has type <type 'numpy.ndarray'>
values has len 14
values is [ 0.   0.3  0.6  0.9  1.2  1.5  1.8  2.1  2.4  2.7  3.   3.3  3.6  3.9]
*In my case, I put numpy into a virtualenv as follows - this should be unnecessary if you can execute python -c "import numpy; print numpy.get_include()" as suggested by @max:

# virtualenv, pip, path unnecessary if your Python has numpy
virtualenv venv
./venv/bin/pip install -r requirements.txt 
export PATH="$(pwd)/venv/bin:$PATH"
Have fun! :-)

shareimprove this answer
answered Apr 10 '16 at 0:17

sage
2,72512640
add a comment
Your Answer

Sign up or log in
 Sign up using Google
 Sign up using Facebook
 Sign up using Email and Password
Post as a guest
Name

Email
Required, but never shown


Post Your Answer
By clicking "Post Your Answer", you acknowledge that you have read our updated terms of service, privacy policy and cookie policy, and that your continued use of the website is subject to these policies.

Not the answer you're looking for? Browse other questions tagged c++ arrays numpy boost-python or ask your own question.
asked

6 years, 8 months ago

viewed

20,790 times

active

1 month ago

BLOG
Stack Overflow & InfoJobs: Bringing Opportunities to Developers in Spain
Linked
246
A positive lambda: '+[]{}' - What sorcery is this?
3
Exposing OpenCV-based C++ function with Mat/Numpy conversion to Python
2
Passing C structs to python as numpy arrays using boost python
5
How to convert NumPy ndarray to C++ vector with Boost.Python and back?
0
How to convert a vector to numpy array with templates and boost?
1
PyArray_New or PyArray_SimpleNewFromData specify dimensions for a 3D array
0
Smarter way of calling boost::python::call?
1
Python newbie: old C++ code for interfacing with Python uses deprecated boost/python/numeric.hpp instead of NumPy. How to update?
0
How to convert boost::Makevariantover to vector
0
Segmentation Fault on using PyArray_SimpleNewFromData on converting mat to numpyarray
see more linked questions…
Related
2822
What are the differences between a pointer variable and a reference variable in C++?
3179
Create ArrayList from array
3442
How do I check if an array includes an object in JavaScript?
2895
How to append something to an array?
6768
How do I remove a particular element from an array in JavaScript?
2078
Why are elementwise additions much faster in separate loops than in a combined loop?
1608
Why is reading lines from stdin much slower in C++ than Python?
1478
Image Processing: Algorithm Improvement for 'Coca-Cola Can' Recognition
1414
Why should I use a pointer rather than the object itself?
1343
Compiling an application for use in highly radioactive environments
Hot Network Questions
Is it unethical to supply a marked solution to a student who has shown intent to cheat, with the intent of identifying them for punishment?
Does it make sense to consider a triggerable server software crash a DOS attack?
Has it been mathematically proven that antivirus can't detect all viruses?
Why do RPGs let you know how much XP you need to level up?
Is what the Joker says about world wars in "The Killing Joke" true?
Find the needle in the haystack
What program is Neo using?
Do hard to pronounce names break immersion?
Noteworthy, but not so famous conjectures proved recent years
Are 30% of human embryos spontaneously "aborted"?
Solving equations involving square roots
Start enumerating an item in the same line after a exercise number
Firefox Quantum : HTML files from internal storage (/usr) do not open
Dominant contribution of ν = 0 → ν = 1 transition in absorption spectra when studying vibrational spectroscopy
Will the PhD degree be retracted if the thesis is found to be not novel?
How to reference the result of reduce() operation in Java 8?
Post 1999 will anyone be entitled to sit in the House of Lords by heredity alone?
How could intelligent animals defend against humans?
Are there linux distributions/processes where upgrades/patches never require reboots? Is livepatching practical in 2019?
Sum of Powers of 2
Letter of support for promotion to full professor from an "unusual" student
Did Phil retain the skills he learned?
Am I conveying disrespect if I omit my gender pronoun from a conference nametag?
Is Vishvaksena mentioned in any major scriptures?
 question feed
STACK OVERFLOW
Questions
Jobs
Developer Jobs Directory
Salary Calculator
Help
Mobile
Disable Responsiveness
PRODUCTS
Teams
Talent
Engagement
Enterprise
COMPANY
About
Press
Work Here
Legal
Privacy Policy
Contact Us
STACK EXCHANGE
NETWORK
Technology
Life / Arts
Culture / Recreation
Science
Other
Blog
Facebook
Twitter
LinkedIn
site design / logo © 2019 Stack Exchange Inc; user contributions licensed under cc by-sa 3.0 with attribution required. rev 2019.2.1.32789
