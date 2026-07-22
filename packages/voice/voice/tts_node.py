import sys
sys.path.insert(0, '/root/venv/lib/python3.12/site-packages')

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import wave
import io
import grpc
import yandex.cloud.ai.tts.v3.tts_pb2 as tts_pb2
import yandex.cloud.ai.tts.v3.tts_service_pb2_grpc as tts_service_pb2_grpc

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')

        # Параметры
        self.declare_parameter('iam_token', '')
        self.declare_parameter('api_key_file', '/src/rover_m2m/rover-m2m.env')
        self.declare_parameter('folder_id', '')
        self.declare_parameter('voice', 'alena')  # filipp, ermil, jane, alena
        self.declare_parameter('speed', 1.0)

        # Получение параметров
        self.iam_token = self.get_parameter('iam_token').value
        self.folder_id = self.get_parameter('folder_id').value
        self.api_key = self.read_api_key(self.get_parameter('api_key_file').value)

        if not self.iam_token and not self.api_key:
            self.get_logger().error('Neither IAM token nor API key provided!')
            return

        # Subscriber на текст для озвучивания
        self.text_sub = self.create_subscription(
            String,
            'voice/speak',
            self.speak_callback,
            10
        )

        # PyAudio для воспроизведения
        self.audio = pyaudio.PyAudio()

        self.get_logger().info('TTS Node started (API v3)')

    @staticmethod
    def read_api_key(path):
        try:
            with open(path, 'r', encoding='utf-8') as config:
                for line in config:
                    if line.startswith('YANDEX_API_KEY='):
                        return line.split('=', 1)[1].strip().strip('"\'')
        except OSError:
            pass
        return ''

    def speak_callback(self, msg):
        """Синтез и воспроизведение речи"""
        text = msg.data

        if not text:
            return

        self.get_logger().info(f'Speaking: {text}')

        try:
            # Синтез речи
            audio_data = self.synthesize_speech(text)

            # Воспроизведение
            self.play_audio(audio_data)

        except Exception as e:
            self.get_logger().error(f'TTS error: {e}')

    def synthesize_speech(self, text):
        """Синтез речи через SpeechKit API v3"""

        # Установка соединения с сервером
        cred = grpc.ssl_channel_credentials()
        channel = grpc.secure_channel('tts.api.cloud.yandex.net:443', cred)
        stub = tts_service_pb2_grpc.SynthesizerStub(channel)

        # Подготовка метаданных для авторизации
        if self.api_key:
            metadata_list = [('authorization', f'Api-Key {self.api_key}')]
        else:
            metadata_list = [('authorization', f'Bearer {self.iam_token}')]

        # Folder ID is used with IAM tokens. API-key requests infer its folder.
        if self.folder_id and not self.api_key:
            metadata_list.append(('x-folder-id', self.folder_id))

        # Параметры синтеза
        request = tts_pb2.UtteranceSynthesisRequest(
            text=text,
            output_audio_spec=tts_pb2.AudioFormatOptions(
                container_audio=tts_pb2.ContainerAudio(
                    container_audio_type=tts_pb2.ContainerAudio.WAV
                )
            ),
            hints=[
                tts_pb2.Hints(
                    voice=self.get_parameter('voice').value,
                    speed=self.get_parameter('speed').value
                )
            ],
            loudness_normalization_type=tts_pb2.UtteranceSynthesisRequest.LUFS
        )

        # Получение аудио потока
        response_stream = stub.UtteranceSynthesis(request, metadata=tuple(metadata_list))

        # Сборка аудио
        audio_data = b''
        for response in response_stream:
            audio_data += response.audio_chunk.data

        return audio_data

    def play_audio(self, audio_data):
        """Воспроизведение аудио"""

        try:
            # Парсинг WAV
            with io.BytesIO(audio_data) as audio_file:
                with wave.open(audio_file, 'rb') as wf:
                    # Параметры аудио
                    channels = wf.getnchannels()
                    sample_width = wf.getsampwidth()
                    framerate = wf.getframerate()

                    # Открытие потока
                    stream = self.audio.open(
                        format=self.audio.get_format_from_width(sample_width),
                        channels=channels,
                        rate=framerate,
                        output=True
                    )

                    # Воспроизведение
                    chunk_size = 1024
                    data = wf.readframes(chunk_size)

                    while data:
                        stream.write(data)
                        data = wf.readframes(chunk_size)

                    # Закрытие потока
                    stream.stop_stream()
                    stream.close()
        except Exception as e:
            self.get_logger().error(f'Audio playback error: {e}')

    def destroy_node(self):
        """Корректное завершение"""
        self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
