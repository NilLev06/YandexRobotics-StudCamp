import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import queue
import threading
import grpc
import yandex.cloud.ai.stt.v3.stt_pb2 as stt_pb2
import yandex.cloud.ai.stt.v3.stt_service_pb2_grpc as stt_service_pb2_grpc

class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')

        # Параметры
        self.declare_parameter('iam_token', '')
        self.declare_parameter('folder_id', '')
        self.declare_parameter('language', 'ru-RU')
        self.declare_parameter('sample_rate', 8000)

        # Получение параметров
        self.iam_token = self.get_parameter('iam_token').value
        self.folder_id = self.get_parameter('folder_id').value

        if not self.iam_token:
            self.get_logger().error('IAM token not provided!')
            return

        # Publisher для распознанного текста
        self.text_pub = self.create_publisher(
            String,
            'voice/recognized_text',
            10
        )

        # Параметры аудио
        self.CHUNK = 4000
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = self.get_parameter('sample_rate').value

        # Очередь для аудио чанков
        self.audio_queue = queue.Queue()

        # Инициализация PyAudio
        self.audio = pyaudio.PyAudio()

        # Запуск потоков
        self.recording = True
        self.record_thread = threading.Thread(target=self.record_audio)
        self.recognize_thread = threading.Thread(target=self.recognize_stream)

        self.record_thread.start()
        self.recognize_thread.start()

        self.get_logger().info('STT Node started (API v3)')

    def record_audio(self):
        """Запись аудио с микрофона"""
        try:
            stream = self.audio.open(
                format=self.FORMAT,
                channels=self.CHANNELS,
                rate=self.RATE,
                input=True,
                frames_per_buffer=self.CHUNK
            )

            self.get_logger().info('Recording started...')

            while self.recording:
                try:
                    data = stream.read(self.CHUNK, exception_on_overflow=False)
                    self.audio_queue.put(data)
                except Exception as e:
                    self.get_logger().error(f'Recording error: {e}')

            stream.stop_stream()
            stream.close()
        except Exception as e:
            self.get_logger().error(f'Failed to open audio stream: {e}')

    def audio_generator(self):
        """Генератор аудио чанков для streaming (API v3)"""
        # Первое сообщение - настройки распознавания
        recognize_options = stt_pb2.StreamingOptions(
            recognition_model=stt_pb2.RecognitionModelOptions(
                audio_format=stt_pb2.AudioFormatOptions(
                    raw_audio=stt_pb2.RawAudio(
                        audio_encoding=stt_pb2.RawAudio.LINEAR16_PCM,
                        sample_rate_hertz=self.RATE,
                        audio_channel_count=self.CHANNELS
                    )
                ),
                text_normalization=stt_pb2.TextNormalizationOptions(
                    text_normalization=stt_pb2.TextNormalizationOptions.TEXT_NORMALIZATION_ENABLED,
                    profanity_filter=False,
                    literature_text=False
                ),
                language_restriction=stt_pb2.LanguageRestrictionOptions(
                    restriction_type=stt_pb2.LanguageRestrictionOptions.WHITELIST,
                    language_code=[self.get_parameter('language').value]
                ),
                audio_processing_type=stt_pb2.RecognitionModelOptions.REAL_TIME
            )
        )

        yield stt_pb2.StreamingRequest(session_options=recognize_options)

        # Затем отправляем аудио чанки
        while self.recording:
            try:
                chunk = self.audio_queue.get(timeout=1.0)
                yield stt_pb2.StreamingRequest(chunk=stt_pb2.AudioChunk(data=chunk))
            except queue.Empty:
                continue

    def recognize_stream(self):
        """Streaming распознавание через SpeechKit API v3"""

        # Установка соединения с сервером
        cred = grpc.ssl_channel_credentials()
        channel = grpc.secure_channel('stt.api.cloud.yandex.net:443', cred)
        stub = stt_service_pb2_grpc.RecognizerStub(channel)

        # Подготовка метаданных для авторизации
        metadata_list = [
            ('authorization', f'Bearer {self.iam_token}'),
        ]

        # Добавляем folder_id если он указан
        if self.folder_id:
            metadata_list.append(('x-folder-id', self.folder_id))

        # Streaming распознавание
        try:
            it = stub.RecognizeStreaming(
                self.audio_generator(),
                metadata=tuple(metadata_list)
            )

            for response in it:
                event_type = response.WhichOneof('Event')

                # Обрабатываем только финальные результаты
                if event_type == 'final' and len(response.final.alternatives) > 0:
                    text = response.final.alternatives[0].text
                    if text:
                        self.get_logger().info(f'Recognized: {text}')

                        # Публикация
                        msg = String()
                        msg.data = text
                        self.text_pub.publish(msg)

                # Промежуточные результаты (для отладки)
                elif event_type == 'partial' and len(response.partial.alternatives) > 0:
                    text = response.partial.alternatives[0].text
                    self.get_logger().debug(f'Partial: {text}')

        except grpc._channel._Rendezvous as err:
            self.get_logger().error(f'Recognition error: {err._state.code} - {err._state.details}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')

    def destroy_node(self):
        """Корректное завершение"""
        self.recording = False
        if hasattr(self, 'record_thread'):
            self.record_thread.join(timeout=2.0)
        if hasattr(self, 'recognize_thread'):
            self.recognize_thread.join(timeout=2.0)
        self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = STTNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
