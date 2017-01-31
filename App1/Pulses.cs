﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace App1
{
    class Pulses
    {
        private const int MAX_PULSES = 314;

        #region SINUS PULSE
        private int[] pulse_sinus = new int[] {0,
5,
10,
15,
20,
25,
31,
36,
41,
46,
51,
56,
61,
66,
70,
75,
80,
85,
90,
95,
99,
104,
109,
113,
118,
122,
127,
131,
135,
140,
144,
148,
152,
156,
160,
164,
168,
172,
176,
179,
183,
186,
190,
193,
197,
200,
203,
206,
209,
212,
215,
217,
220,
222,
225,
227,
230,
232,
234,
236,
238,
239,
241,
243,
244,
246,
247,
248,
249,
250,
251,
252,
253,
253,
254,
254,
255,
255,
255,
255,
255,
255,
254,
254,
253,
253,
252,
251,
250,
249,
248,
247,
246,
244,
243,
241,
240,
238,
236,
234,
232,
230,
227,
225,
223,
220,
217,
215,
212,
209,
206,
203,
200,
197,
194,
190,
187,
183,
180,
176,
172,
168,
165,
161,
157,
153,
148,
144,
140,
136,
131,
127,
123,
118,
114,
109,
104,
100,
95,
90,
85,
81,
76,
71,
66,
61,
56,
51,
46,
41,
36,
31,
26,
21,
16,
11,
6,
0,
-5,
-10,
-15,
-20,
-25,
-30,
-35,
-40,
-45,
-50,
-55,
-60,
-65,
-70,
-75,
-80,
-85,
-89,
-94,
-99,
-104,
-108,
-113,
-117,
-122,
-126,
-131,
-135,
-139,
-144,
-148,
-152,
-156,
-160,
-164,
-168,
-172,
-175,
-179,
-183,
-186,
-190,
-193,
-196,
-199,
-203,
-206,
-209,
-212,
-214,
-217,
-220,
-222,
-225,
-227,
-229,
-232,
-234,
-236,
-238,
-239,
-241,
-243,
-244,
-246,
-247,
-248,
-249,
-250,
-251,
-252,
-253,
-253,
-254,
-254,
-255,
-255,
-255,
-255,
-255,
-255,
-254,
-254,
-254,
-253,
-252,
-251,
-251,
-250,
-248,
-247,
-246,
-245,
-243,
-241,
-240,
-238,
-236,
-234,
-232,
-230,
-228,
-225,
-223,
-220,
-218,
-215,
-212,
-209,
-206,
-203,
-200,
-197,
-194,
-190,
-187,
-183,
-180,
-176,
-173,
-169,
-165,
-161,
-157,
-153,
-149,
-145,
-140,
-136,
-132,
-127,
-123,
-118,
-114,
-109,
-105,
-100,
-95,
-91,
-86,
-81,
-76,
-71,
-66,
-61,
-56,
-51,
-46,
-41,
-36,
-31,
-26,
-21,
-16,
-11,
-6,
-1,
};
        #endregion

        public int[] Pulse_sinus
        {
            get
            {
                return pulse_sinus;
            }

            set
            {
                pulse_sinus = value;
            }
        }
    }
}