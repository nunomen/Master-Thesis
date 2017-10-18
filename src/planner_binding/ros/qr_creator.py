import sys
import binding


def parse_observation_file(input_file_name: str, output_file_name: str) -> None:
    """ Receives path of the file to parse and exports a QR code from that same file

    """
    observations = binding.Observations()

    with open('../instances/' + input_file_name + '.txt') as document:
        for line in document:
            line = line.replace(' ', '')
            line = line.replace('\n', '')
            w = line.split(':')
            if len(w) == 2:
                observations.append_observation(w[0], w[1])

    observations.generate_qrcode(output_file_name)

if __name__ == '__main__':
    args = sys.argv

    if len(args) == 3:
        parse_observation_file(args[1], args[2])
