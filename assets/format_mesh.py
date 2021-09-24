import argparse

def main(args):
    out_str = []
    files = []
    convex_counter = 0
    with open(args.input, 'r') as object:
        lines = object.readlines()
        for i, line in enumerate(lines):
            words = line.split(' ')
            if words[0] == 'o':
                convex_counter = convex_counter + 1
                file = open(args.output_dir + 'pipe{}.txt'.format(convex_counter),'w+')
                files.append(file)
            if words[0] == 'v':
                file.writelines((word + ' ') for word in words[1:4])
    for file in files:
        file.close()
                
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Prepare the .obj file for matlab. \
                                     \nYou have to copy the content in .obj to a .txt file before running this program.")
    parser.add_argument('--input',help='File path of the .txt object.',default='./Pipe/Pipe_convex.txt')
    parser.add_argument('--output_dir',help='File path for formated .txt file.',default='./Pipe/Pipe_mat/')
    args = parser.parse_args()
    main(args)