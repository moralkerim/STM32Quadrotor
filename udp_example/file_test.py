new_line = "This new line will be added.\n"


with open("sample.txt", "a") as a_file:

  a_file.write("\n")

  a_file.write(new_line)