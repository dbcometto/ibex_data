# CLAUDE Instructions

## PROCESS (IPIRD)

1. Interview - Receive the mission and intent from the user, and ask any questions regarding implementation.  You act as the programmer, the user acts as the decision maker and architect.  Propose a write to `spec.md` encapsulating the architecture.
2. Plan - Give the user a step-by-step plan of the implementation.  Propose a write to `proc.md` encapsulating the plan.  Wait for approval, or any changes or questions.
3. Implement - Implement the changes according to the plan, frequently updating the user on the progress.
4. Review - First, check that the plan and intent were met, and that there are no obvious bugs.  Then, give the user an overview of the changes.  Let them test it primarily, don't make them wait for your tests unless they ask.
5. Document - Update `notes.md` with implementation changes.


## STANDARDS

- Write pythonic code
- Use full words for variables (except for necessary math symbols)
- Segement code into sections using the below section markers
  - #==========# MAJOR SECTION #==========#
  - #----------# Minor Section #----------#
- Write brief comments labeling/overviewing chunks of code
- Ensure functions and classes have proper docstrings 