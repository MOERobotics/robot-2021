package frc.robot;

import java.time.LocalDateTime;
import java.time.ZoneOffset;
import java.time.format.DateTimeFormatter;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.locks.ReentrantLock;

public class Logger {

	private static class LoggedMessage {
		public final String        key;
		public final String        message;
		public final Integer       hash;
		public final LocalDateTime expiryTime;
		public LoggedMessage(
			String key,
			String message,
			Integer hash,
			LocalDateTime expiryTime
		) {
			this.key = key;
			this.message = message;
			this.hash = hash;
			this.expiryTime = expiryTime;
		}

		@Override
		public String toString() {
			return String.format(
				"%16.16s | %8.8s | %12.12s | %s",
				key,
				hashString(),
				expiryTimeString(),
				message
			);
		}

		public String hashString() {
			return (hash == null)
				? "--NULL--"
				: String.format("%08X", hash);
		}

		public static final DateTimeFormatter timeFormatter
			= DateTimeFormatter.ofPattern("HH:mm:ss.SSS");

		public String expiryTimeString() {
			return (expiryTime == null)
				? "Does Not Expire"
				: timeFormatter.format(expiryTime);
		}

	}

	private static final LoggedMessage NullLoggedMessage = new LoggedMessage(
		"NULL",
		"NULL",
		null,
		null
	);

	public static final Logger instance = new Logger();

	private ReentrantLock lock = new ReentrantLock();
	private Map<String, LoggedMessage> loggedMessages = new HashMap<>();

	private void logInternal(
		String key,
		String message,
		Integer hash,
		Integer ttl_ms
	) {
		var now = LocalDateTime.now();
		LocalDateTime expiryTime = null;
		if (ttl_ms != null) {
			long ttl_ns = ttl_ms * 1000000;
			expiryTime = now.plusNanos(ttl_ns);
		}
		var newMessage = new LoggedMessage(
			key,
			message,
			hash,
			expiryTime
		);
		try {
			lock.lock();
			if (!loggedMessages.containsKey(key)) {
				loggedMessages.put(key,NullLoggedMessage);
			}

			var oldMessage = loggedMessages.get(key);

			if (checkMessageExpired(oldMessage,newMessage) == true) {
				System.out.printf(
					"LOGGER: %12.12s | %16.16s | %8.8s | %s\n",
					LoggedMessage.timeFormatter.format(now),
					key,
					newMessage.hashString(),
					message
				);
				loggedMessages.put(key,newMessage);
			}

		} finally {
			lock.unlock();
		}
	}

	private static boolean checkMessageExpired (
		LoggedMessage oldMsg,
		LoggedMessage newMsg
	) {
		//NULL message case
		if (oldMsg.key != newMsg.key) return true;
		//Changed value case
		if (
			oldMsg.hash != null &&
			newMsg.hash != null &&
			oldMsg.hash.compareTo(newMsg.hash) != 0
		) return true;
		//Datetime expired case
		if (
			oldMsg.expiryTime != null &&
			oldMsg.expiryTime.isBefore(LocalDateTime.now())
		) return true;
		//Default
		return false;
	}

	public static void log(String key, String message) {
		instance.logInternal(key,message,null,-1);
	}
	public static void logOnce(String key, String message) {
		instance.logInternal(key,message,null,null);
	}
	public static void logTTL(String key, String message, int ttl) {
		instance.logInternal(key,message,null,ttl);
	}
	public static void logValue(String key, String message, long value) {
		instance.logInternal(key,message,Long.hashCode(value),null);
	}
	public static void logValue(String key, String message, double value) {
		instance.logInternal(key,message,Double.hashCode(value),null);
	}
	public static void logValue(String key, String message, Object value) {
		instance.logInternal(key,message, Objects.hashCode(value),null);
	}
	public static void logValueTTL(String key, String message, long value, int ttl) {
		instance.logInternal(key,message,Long.hashCode(value),null);
	}
	public static void logValueTTL(String key, String message, double value, int ttl) {
		instance.logInternal(key,message,Double.hashCode(value),null);
	}
	public static void logValueTTL(String key, String message, Object value, int ttl) {
		instance.logInternal(key,message, Objects.hashCode(value),null);
	}


}
